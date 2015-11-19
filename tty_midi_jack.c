/*
    This file is part of ttymidi.

    ttymidi is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ttymidi is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ttymidi.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <argp.h>
#include <signal.h>
#include <pthread.h>
// Linux-specific
#include <linux/serial.h>
#include <linux/ioctl.h>
#include <asm/ioctls.h>
#include <jack/jack.h>
#include <jack/midiport.h>

#define FALSE                         0
#define TRUE                          1

#define MAX_DEV_STR_LEN               32
#define MAX_MSG_SIZE                1024

/* change this definition for the correct port */
//#define _POSIX_SOURCE 1 /* POSIX compliant source */

int run;
int serial;

/* --------------------------------------------------------------------- */
// Program options

static struct argp_option options[] = 
{
	{"serialdevice" , 's', "DEV" , 0, "Serial device to use. Default = /dev/ttyUSB0" },
	{"baudrate"     , 'b', "BAUD", 0, "Serial port baud rate. Default = 115200" },
	{"verbose"      , 'v', 0     , 0, "For debugging: Produce verbose output" },
	{"printonly"    , 'p', 0     , 0, "Super debugging: Print values read from serial -- and do nothing else" },
	{"quiet"        , 'q', 0     , 0, "Don't produce any output, even when the print command is sent" },
	{"name"		, 'n', "NAME", 0, "Name of the Alsa MIDI client. Default = ttymidi" },
	{ 0 }
};

typedef struct _arguments
{
	int  silent, verbose, printonly;
	char serialdevice[MAX_DEV_STR_LEN];
	int  baudrate;
	char name[MAX_DEV_STR_LEN];
} arguments_t;

typedef struct s_mididata_list *Mididata_list;
struct s_mididata_list {
        uint8_t         data[3];
        uint8_t         length;
        Mididata_list	next;
};

static pthread_mutex_t jack_lock = PTHREAD_MUTEX_INITIALIZER;
typedef struct _jack
{
	jack_client_t *client;
	jack_port_t *midi_out_port;
	jack_port_t *midi_in_port;

	Mididata_list tty_in_data;
	Mididata_list tty_out_data;

	pthread_cond_t data_ready;
} jack_t;

static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
	/* Get the input argument from argp_parse, which we
	   know is a pointer to our arguments structure. */
	arguments_t *arguments = state->input;
	int baud_temp;

	switch (key)
	{
		case 'p':
			arguments->printonly = 1;
			break;
		case 'q':
			arguments->silent = 1;
			break;
		case 'v':
			arguments->verbose = 1;
			break;
		case 's':
			if (arg == NULL) break;
			strncpy(arguments->serialdevice, arg, MAX_DEV_STR_LEN);
			break;
		case 'n':
			if (arg == NULL) break;
			strncpy(arguments->name, arg, MAX_DEV_STR_LEN);
			break;
		case 'b':
			if (arg == NULL) break;
			baud_temp = strtol(arg, NULL, 0);
			if (baud_temp != EINVAL && baud_temp != ERANGE)
				switch (baud_temp)
				{
					case 1200   : arguments->baudrate = B1200  ; break;
					case 2400   : arguments->baudrate = B2400  ; break;
					case 4800   : arguments->baudrate = B4800  ; break;
					case 9600   : arguments->baudrate = B9600  ; break;
					case 19200  : arguments->baudrate = B19200 ; break;
					case 38400  : arguments->baudrate = B38400 ; break;
					case 57600  : arguments->baudrate = B57600 ; break;
					case 115200 : arguments->baudrate = B115200; break;
					default: printf("Baud rate %i is not supported.\n",baud_temp); exit(1);
				}

		case ARGP_KEY_ARG:
		case ARGP_KEY_END:
			break;

		default:
			return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

void arg_set_defaults(arguments_t *arguments)
{
	char *serialdevice_temp = "/dev/ttyUSB0";
	arguments->printonly    = 0;
	arguments->silent       = 0;
	arguments->verbose      = 0;
	arguments->baudrate     = B115200;
	char *name_tmp		= (char *)"ttymidi";
	strncpy(arguments->serialdevice, serialdevice_temp, MAX_DEV_STR_LEN);
	strncpy(arguments->name, name_tmp, MAX_DEV_STR_LEN);
}

const char *argp_program_version     = "ttymidi 0.60";
const char *argp_program_bug_address = "tvst@hotmail.com";
static char doc[]       = "ttymidi - Connect serial port devices to ALSA MIDI programs!";
static struct argp argp = { options, parse_opt, 0, doc };
static arguments_t arguments;
static jack_t jack;

static void add_midi_event(Mididata_list *midi_list, uint8_t *data,
                                            int size) {
        Mididata_list cur_data;

	Mididata_list tmp_data = malloc(sizeof(struct s_mididata_list));

	pthread_mutex_lock(&jack_lock);

        if (!*midi_list) {
            *midi_list = tmp_data;
            cur_data = *midi_list;
        } else {
            cur_data = *midi_list;
            while (cur_data->next) cur_data = cur_data->next;
            cur_data->next = tmp_data;
            cur_data = cur_data->next;
        }
        memcpy(cur_data->data, data, size);
	cur_data->length = size;
	cur_data->next = NULL;

	pthread_mutex_unlock(&jack_lock);
}

static int jack_callback(jack_nframes_t nframes, void *arg) {
        jack_midi_event_t jack_midi_event;
        jack_nframes_t event_index = 0;
        Mididata_list cur_data;

        void *midi_in_buf = jack_port_get_buffer(jack.midi_in_port, nframes);
        void *midi_out_buf = jack_port_get_buffer(jack.midi_out_port, nframes);

        jack_midi_clear_buffer(midi_out_buf);

	pthread_mutex_lock(&jack_lock);
        while (jack.tty_in_data) {
            cur_data = jack.tty_in_data;
            jack.tty_in_data = jack.tty_in_data->next;

	    pthread_mutex_unlock(&jack_lock);

            jack_midi_event_write(midi_out_buf, event_index++, cur_data->data,
                                cur_data->length);
            free(cur_data);

	    pthread_mutex_lock(&jack_lock);
        }
	pthread_mutex_unlock(&jack_lock);

        event_index = 0;

        while (jack_midi_event_get(&jack_midi_event, midi_in_buf,
                                        event_index++) == 0) {
            add_midi_event(&jack.tty_out_data, jack_midi_event.buffer,
                                    jack_midi_event.size);
            pthread_cond_signal(&jack.data_ready);
        }

        return 0;
}

static int jack_close(void) {
        jack_deactivate(jack.client);

        if (jack.midi_out_port)
            jack_port_unregister(jack.client, jack.midi_out_port);
        if (jack.midi_in_port)
            jack_port_unregister(jack.client, jack.midi_in_port);

        return jack_client_close(jack.client);
}

void exit_cli(int sig)
{
	printf("\rttymidi closing down ... ");
	jack_close();
	run = FALSE;
        pthread_cond_signal(&jack.data_ready);
}

/* --------------------------------------------------------------------- */
// MIDI stuff

int open_seq(void) 
{
	jack_status_t jack_status;
	jack.client = jack_client_open(arguments.name,
			JackNoStartServer, &jack_status);

	if (!jack.client) return -1;

	jack.midi_out_port = jack_port_register(jack.client, "tty_in",
			JACK_DEFAULT_MIDI_TYPE,
			JackPortIsOutput | JackPortIsTerminal, 0);

	jack.midi_in_port = jack_port_register(jack.client, "tty_out",
			JACK_DEFAULT_MIDI_TYPE,
			JackPortIsInput | JackPortIsTerminal, 0);

	if (!(jack.midi_out_port && jack.midi_in_port)) return -1;

	jack_set_process_callback(jack.client, jack_callback, 0);

	if (jack_activate(jack.client)) return -1;

	pthread_cond_init(&jack.data_ready, NULL);

	return 0;
}

void parse_midi_command(char *buf)
{
	/*
	   MIDI COMMANDS
	   -------------------------------------------------------------------
	   name                 status      param 1          param 2
	   -------------------------------------------------------------------
	   note off             0x80+C       key #            velocity
	   note on              0x90+C       key #            velocity
	   poly key pressure    0xA0+C       key #            pressure value
	   control change       0xB0+C       control #        control value
	   program change       0xC0+C       program #        --
	   mono key pressure    0xD0+C       pressure value   --
	   pitch bend           0xE0+C       range (LSB)      range (MSB)
	   system               0xF0+C       manufacturer     model
	   -------------------------------------------------------------------
	   C is the channel number, from 0 to 15;
	   -------------------------------------------------------------------
	   source: http://ftp.ec.vanderbilt.edu/computermusic/musc216site/MIDI.Commands.html
	
	   In this program the pitch bend range will be transmitter as 
	   one single 8-bit number. So the end result is that MIDI commands 
	   will be transmitted as 3 bytes, starting with the operation byte:
	
	   buf[0] --> operation/channel
	   buf[1] --> param1
	   buf[2] --> param2        (param2 not transmitted on program change or key press)
   */

	int operation, channel, param1, param2;

	operation = buf[0] & 0xF0;
	channel   = buf[0] & 0x0F;
	param1    = buf[1];
	param2    = buf[2];

	switch (operation)
	{
		case 0x80:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Note off           %03u %03u %03u\n", operation, channel, param1, param2);
			add_midi_event(&jack.tty_in_data, buf, 3);
			break;
			
		case 0x90:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Note on            %03u %03u %03u\n", operation, channel, param1, param2);
			add_midi_event(&jack.tty_in_data, buf, 3);
			break;
			
		case 0xA0:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Pressure change    %03u %03u %03u\n", operation, channel, param1, param2);
			add_midi_event(&jack.tty_in_data, buf, 3);
			break;

		case 0xB0:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Controller change  %03u %03u %03u\n", operation, channel, param1, param2);
			add_midi_event(&jack.tty_in_data, buf, 3);
			break;

		case 0xC0:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Program change     %03u %03u\n", operation, channel, param1);
			add_midi_event(&jack.tty_in_data, buf, 2);
			break;

		case 0xD0:
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Channel change     %03u %03u\n", operation, channel, param1);
			add_midi_event(&jack.tty_in_data, buf, 2);
			break;

		case 0xE0:
			param1 = (param1 & 0x7F) + ((param2 & 0x7F) << 7);
			if (!arguments.silent && arguments.verbose) 
				printf("Serial  0x%x Pitch bend         %03u %05i\n", operation, channel, param1);
			add_midi_event(&jack.tty_in_data, buf, 3);
			break;

		/* Not implementing system commands (0xF0) */
			
		default:
			if (!arguments.silent) 
				printf("0x%x Unknown MIDI cmd   %03u %03u %03u\n", operation, channel, param1, param2);
			break;
	}

}

void write_midi_action_to_serial_port(Mididata_list data) 
{
	uint8_t bytes[3];

	uint8_t type = data->data[0] & 0xf0;

	memcpy(bytes, data->data, data->length);

	switch (type) 
	{

		case 0x80: 
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Note off           %03u %03u %03u\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]); 
			break; 

		case 0x90:
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Note on            %03u %03u %03u\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]); 
			break;        

		case 0xA0:
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Pressure change    %03u %03u %03u\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]); 
			break;       

		case 0xB0:
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Controller change  %03u %03u %03u\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]); 
			break;   

		case 0xC0:
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Program change     %03u %03u \n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1]); 
			break;  

		case 0xD0: 
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Channel change     %03u %03u \n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1]); 
			break;  

		case 0xE0:
			if (!arguments.silent && arguments.verbose) 
				printf("Alsa    0x%x Pitch bend         %03u %5d\n", bytes[0]&0xF0, bytes[0]&0xF, ((uint16_t) (bytes[2] << 7)) | bytes[1]);
			break;

		default:
			break;
	}

	write(serial, data->data, data->length);
}

void* read_midi_from_jack(void* unused) 
{
	Mididata_list cur_data;
	while (run) 
	{
		pthread_mutex_lock(&jack_lock);
		while (jack.tty_out_data == NULL) {
			pthread_cond_wait(&jack.data_ready, &jack_lock);
			if (!run) return;
		}

		cur_data = jack.tty_out_data;
		jack.tty_out_data = jack.tty_out_data->next;

		pthread_mutex_unlock(&jack_lock);

		write_midi_action_to_serial_port(cur_data);

		free(cur_data);
	}	

	printf("\nStopping [PC]->[Hardware] communication...");
}

void* read_midi_from_serial_port(void* unused) 
{
	char buf[3], msg[MAX_MSG_SIZE];
	int i, msglen;
	
	/* Lets first fast forward to first status byte... */
	if (!arguments.printonly) {
		do read(serial, buf, 1);
		while (buf[0] >> 7 == 0);
	}

	while (run) 
	{
		/* 
		 * super-debug mode: only print to screen whatever
		 * comes through the serial port.
		 */

		if (arguments.printonly) 
		{
			read(serial, buf, 1);
			printf("%x\t", (int) buf[0]&0xFF);
			fflush(stdout);
			continue;
		}

		/* 
		 * so let's align to the beginning of a midi command.
		 */

		int i = 1;

		while (i < 3) {
			read(serial, buf+i, 1);

			if (buf[i] >> 7 != 0) {
				/* Status byte received and will always be first bit!*/
				buf[0] = buf[i];
				i = 1;
			} else {
				/* Data byte received */
				if (i == 2) {
					/* It was 2nd data byte so we have a MIDI event
					   process! */
					i = 3;
				} else {
					/* Lets figure out are we done or should we read one more byte. */
					if ((buf[0] & 0xF0) == 0xC0 || (buf[0] & 0xF0) == 0xD0) {
						i = 3;
					} else {
						i = 2;
					}
				}
			}

		}

		/* print comment message (the ones that start with 0xFF 0x00 0x00 */
		if (buf[0] == (char) 0xFF && buf[1] == (char) 0x00 && buf[2] == (char) 0x00)
		{
			read(serial, buf, 1);
			msglen = buf[0];
			if (msglen > MAX_MSG_SIZE-1) msglen = MAX_MSG_SIZE-1;

			read(serial, msg, msglen);

			if (arguments.silent) continue;

			/* make sure the string ends with a null character */
			msg[msglen] = 0;

			puts("0xFF Non-MIDI message: ");
			puts(msg);
			putchar('\n');
			fflush(stdout);
		}

		/* parse MIDI message */
		else parse_midi_command(buf);
	}
}

/* --------------------------------------------------------------------- */
// Main program

main(int argc, char** argv)
{
	//arguments arguments;
	struct termios oldtio, newtio;
	struct serial_struct ser_info;
	char* modem_device = "/dev/ttyS0";

	arg_set_defaults(&arguments);
	argp_parse(&argp, argc, argv, 0, 0, &arguments);

	/*
	 * Open MIDI output port
	 */

	if (open_seq() < 0) {
	    printf("Couldn't connect to Jack.\n");
	    exit(-1);
	}

	/* 
	 *  Open modem device for reading and not as controlling tty because we don't
	 *  want to get killed if linenoise sends CTRL-C.
	 */
	
	serial = open(arguments.serialdevice, O_RDWR | O_NOCTTY ); 

	if (serial < 0) 
	{
		perror(arguments.serialdevice); 
		exit(-1); 
	}

	/* save current serial port settings */
	tcgetattr(serial, &oldtio); 

	/* clear struct for new port settings */
	bzero(&newtio, sizeof(newtio)); 

	/* 
	 * BAUDRATE : Set bps rate. You could also use cfsetispeed and cfsetospeed.
	 * CRTSCTS  : output hardware flow control (only used if the cable has
	 * all necessary lines. See sect. 7 of Serial-HOWTO)
	 * CS8      : 8n1 (8bit, no parity, 1 stopbit)
	 * CLOCAL   : local connection, no modem contol
	 * CREAD    : enable receiving characters
	 */
	newtio.c_cflag = arguments.baudrate | CS8 | CLOCAL | CREAD; // CRTSCTS removed

	/*
	 * IGNPAR  : ignore bytes with parity errors
	 * ICRNL   : map CR to NL (otherwise a CR input on the other computer
	 * will not terminate input)
	 * otherwise make device raw (no other input processing)
	 */
	newtio.c_iflag = IGNPAR;

	/* Raw output */
	newtio.c_oflag = 0;

	/*
	 * ICANON  : enable canonical input
	 * disable all echo functionality, and don't send signals to calling program
	 */
	newtio.c_lflag = 0; // non-canonical

	/* 
	 * set up: we'll be reading 4 bytes at a time.
	 */
	newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 1;     /* blocking read until n character arrives */

	/* 
	 * now clean the modem line and activate the settings for the port
	 */
	tcflush(serial, TCIFLUSH);
	tcsetattr(serial, TCSANOW, &newtio);

	// Linux-specific: enable low latency mode (FTDI "nagling off")
//	ioctl(serial, TIOCGSERIAL, &ser_info);
//	ser_info.flags |= ASYNC_LOW_LATENCY;
//	ioctl(serial, TIOCSSERIAL, &ser_info);

	if (arguments.printonly) 
	{
		printf("Super debug mode: Only printing the signal to screen. Nothing else.\n");
	}

	/* 
	 * read commands
	 */

	/* Starting thread that is polling alsa midi in port */
	pthread_t midi_out_thread, midi_in_thread;
	int iret1, iret2;
	run = TRUE;
	iret1 = pthread_create(&midi_out_thread, NULL, read_midi_from_jack, NULL);
	/* And also thread for polling serial data. As serial is currently read in
           blocking mode, by this we can enable ctrl+c quiting and avoid zombie
           alsa ports when killing app with ctrl+z */
	iret2 = pthread_create(&midi_in_thread, NULL, read_midi_from_serial_port, NULL);
	signal(SIGINT, exit_cli);
	signal(SIGTERM, exit_cli);

	while (run)
	{   
		sleep(100);
	}

	void* status;
	pthread_join(midi_out_thread, &status);

	/* restore the old port settings */
	tcsetattr(serial, TCSANOW, &oldtio);
	printf("\ndone!\n");
}

