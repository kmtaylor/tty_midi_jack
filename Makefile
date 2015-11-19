all:
	gcc tty_midi_jack.c -o tty_midi_jack -lasound -lpthread
clean:
	rm tty_midi_jack
install:
	mkdir -p $(DESTDIR)/usr/bin
	cp tty_midi_jack $(DESTDIR)/usr/bin
uninstall:
	rm $(DESTDIR)/usr/bin/tty_midi_jack
