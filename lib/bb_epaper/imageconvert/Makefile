all: imgconvert

CC     = gcc
CFLAGS = -Wall

imgconvert: main.c
	$(CC) $(CFLAGS) $< -o $@
	strip $@

clean:
	rm -f imgconvert
