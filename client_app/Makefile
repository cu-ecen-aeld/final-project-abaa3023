INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= 

HFILES= 
CFILES= seqv4l2.c capturelib.c client.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	seqv4l2 
	#capturelib

clean:
	-rm -f *.o *.d frames/*.pgm frames/*.ppm
	-rm -f seqgenex0 seqgen seqgen2 seqgen3 seqv4l2 clock_times capture client

seqv4l2: seqv4l2.o capturelib.o client.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o capturelib.o client.o -lpthread -lrt

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

