# (c) Bluemark Innovations BV 
# MIT license

CC=gcc
LD=ld
BINDIR = ./bin
OPTIONS = -O2 -lm -lpthread -I./opendroneid-core-c/mavlink_c_library_v2/ -I./opendroneid-core-c/libopendroneid/ -I./opendroneid-core-c/libmav2odid/ -L./opendroneid-core-c/build/libmav2odid/CMakeFiles/mav2odid.dir/ -L./opendroneid-core-c/build/libopendroneid/CMakeFiles/opendroneid.dir/

all: $(BINDIR)/demo_tx

$(BINDIR)/demo_tx: demo_tx.c
	./build.opendroneid.sh # build opendrone ID to generate required .o files.
	$(CC) demo_tx.c ./opendroneid-core-c/build/libopendroneid/CMakeFiles/opendroneid.dir/opendroneid.c.o -o $(BINDIR)/demo_tx $(OPTIONS) 

clean:
	rm -f $(BINDIR)/demo_tx


