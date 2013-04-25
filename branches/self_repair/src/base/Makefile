SRC = robot.cc robot_ircomm.cc robot_ethcomm.cc robot_option.cc robot_self_repair.cc parameter.cc global.cc IRMessage.cc robot_IPC.cc robot_debug.cc
SRC+= organism.cc organism_sequence.cc organism_sample.cc
SRC+= support.cc worldfile.cc hist.cc ipc.cc

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip

# Change this to your paths
IROBOTPATH = ../../irobot
IPATH = .
IPATH += $(IROBOTPATH)/include

VPATH = .
VPATH += og
VPATH += utils

# Change this to whatever name you like
EXE = morph

static_LIB = libmorph_base.a
shared_LIB = libmorph_base.so

CXXFLAGS =-O2 -Wall -Wno-reorder  -mcpu=bf561 
CFLAGS = -O2 -Wall -Wno-reorder -mcpu=bf561 -std=gnu99 


CXXFLAGS += $(patsubst %, -I%, $(IPATH))
LDFLAGS = -L$(IROBOTPATH)/lib -lpthread -lirobot
LDFLAGS_$(shared_LIB) = -shared -Wl -L$(IROBOTPATH)/lib -lirobot

TOBJECTS = $(SRC:%.cc=objects/%.o)
OBJECTS = $(TOBJECTS:%.c=objects/%.o)

all: objects lib lib/$(static_LIB) lib/$(shared_LIB)

objects:
	mkdir objects

lib:
	mkdir lib

bin:
	mkdir bin

lib/$(static_LIB): $(OBJECTS)
	$(AR) rcu $@ $^
	$(OBJDUMP) -hS $@ > objects/$(static_LIB).lst

lib/$(shared_LIB): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS_$(shared_LIB))
	$(OBJDUMP) -hS $@ > objects/$(shared_LIB).lst


bin/$(EXE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
	$(STRIP) bin/$(EXE)
	$(OBJDUMP) -hS $@ > objects/$(EXE).lst

objects/%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
objects/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
	
clean:
	rm -f lib/$(static_LIB) lib/$(shared_LIB) objects/*.o objects/*.lst
	@mkdir -p objects lib
	rmdir objects lib

