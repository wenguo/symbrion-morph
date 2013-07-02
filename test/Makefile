
SRC = main.cc ipc.cc

# Change this to your path to the irobot directory
IROBOTPATH = ../irobot

# Change this to whatever name you like
EXE = main

CXXFLAGS =	-O2  -Wall -mcpu=bf561
CFLAGS = -O2  -Wall -mcpu=bf561 -std=gnu99

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip

IPATH = .
IPATH += $(IROBOTPATH)/include
IPATH += $(IROBOTPATH)/tools/ipc/ipc

VPATH = .
VPATH += $(IROBOTPATH)/tools/ipc/ipc

CXXFLAGS += $(patsubst %, -I%, $(IPATH))
LDFLAGS = -L$(IROBOTPATH)/lib -lpthread -lirobot

TOBJECTS = $(SRC:%.cc=objects/%.o)
OBJECTS = $(TOBJECTS:%.c=objects/%.o)

all: objects bin bin/$(EXE) strip

objects:
	mkdir objects

bin:
	mkdir bin

bin/$(EXE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

objects/%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
objects/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

objdump:
	$(OBJDUMP) -hS bin/$(EXE) > objects/$(EXE).lst

strip:
	$(STRIP) bin/$(EXE)
	
clean:
	rm -rf bin/$(EXE) $(OBJECTS) objects bin
