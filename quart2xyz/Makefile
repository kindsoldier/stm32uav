

all: main

CFLAGS+= -Wall -O
LDFLAGS+= -lm

OBJS+= main.o


main: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(^F)

exec: main
	./main

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f main
	rm -f *.o $(OBJS)
