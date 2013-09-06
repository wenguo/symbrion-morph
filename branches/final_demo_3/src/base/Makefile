SRC = robot.cc robot_ircomm.cc robot_ethcomm.cc robot_option.cc robot_self_repair.cc parameter.cc global.cc IRMessage.cc robot_IPC.cc robot_debug.cc robot_organism.cc robot_daemon.cc robot_vision.cc
SRC += organism.cc organism_sequence.cc organism_sample.cc
SRC += support.cc worldfile.cc hist.cc ipc.cc
SRC += capture.cc cmvision.cc v4l_helper.cc
SRC += CMessage.cc

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip
SIZE = bfin-linux-uclibc-size

# Change this to your paths
IROBOTPATH = ../../irobot
IPATH = .
IPATH += $(IROBOTPATH)/include
IPATH += ../../cmvision

VPATH = .
VPATH += og
VPATH += utils
VPATH += ../../cmvision

# Change this to whatever name you like
EXE = morph

static_LIB = libmorph_base.a
shared_LIB = libmorph_base.so

CXXFLAGS =-O2 -Wall -Wno-reorder  -mcpu=bf561 -DWITH_MOTION -DWIP -DTEST_CLIMBING
CFLAGS = -O2 -Wall -Wno-reorder -mcpu=bf561 -std=gnu99 -DWITH_MOTION -DWIP -DTEST_CLIMBING

#for cmvision
CXXFLAGS += -DLAPTOP
CFLAGS += -DLAPTOP


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
	$(SIZE) -B $@

lib/$(shared_LIB): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS_$(shared_LIB))
	$(OBJDUMP) -hS $@ > objects/$(shared_LIB).lst
	$(SIZE) -B $@


bin/$(EXE): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
	$(STRIP) bin/$(EXE)
	$(OBJDUMP) -hS $@ > objects/$(EXE).lst
	$(SIZE) -B $@

objects/%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
objects/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
	
clean:
	rm -f lib/$(static_LIB) lib/$(shared_LIB) objects/*.o objects/*.lst
	@mkdir -p objects lib
	rmdir objects lib

