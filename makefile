CFLAGS = -Wl,--no-as-needed -Iinclude -Wno-psabi
LDFLAGS = -lm
.KEEP_STATE:

all:			single_canABH3 bcast_canABH3 single_canABH3++ bcast_canABH3++
clean:
				rm -f *.o single_canABH3 bcast_canABH3 single_canABH3++ bcast_canABH3++

single_canABH3:	single_canABH3.c canABH3.o lib.o
bcast_canABH3:	bcast_canABH3.c canABH3.o lib.o
single_canABH3++:	single_canABH3++.cpp canABH3++.o canABH3.o lib.o
bcast_canABH3++:	bcast_canABH3++.cpp canABH3++.o canABH3.o lib.o
canABH3.o:		canABH3.c canABH3.h lib.o
