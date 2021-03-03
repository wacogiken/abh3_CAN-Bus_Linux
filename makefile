CFLAGS = -Wl,--no-as-needed 
LDFLAGS = -lm
.KEEP_STATE:

all:			single_canABH3 bcast_canABH3
clean:
				rm -f *.o single_canABH3 bcast_canABH3

single_canABH3:	single_canABH3.c canABH3.o pack_float.o lib.o
bcast_canABH3:	bcast_canABH3.c canABH3.o pack_float.o lib.o
canABH3.o:		canABH3.c canABH3.h lib.o
pack_float.o:	pack_float.c pack_float.h _math.h
