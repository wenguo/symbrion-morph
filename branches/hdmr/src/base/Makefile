SRC = robot.cc robot_ircomm.cc robot_ethcomm.cc robot_option.cc robot_self_repair.cc parameter.cc global.cc IRMessage.cc robot_IPC.cc robot_debug.cc robot_organism.cc
SRC+= organism.cc organism_sequence.cc organism_sample.cc
SRC+= Ethernet.cc
SRC+= support.cc worldfile.cc hist.cc ipc.cc lolmsg.c crc8.c bytequeue.c

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip

MIDDLEWARE_PATH_ROOT=$(HOME)

HDMR_PATH=$(MIDDLEWARE_PATH_ROOT)/hdmrplus_install
SOAP_PATH=$(MIDDLEWARE_PATH_ROOT)/gsoap-2.8

MIDDLEWARE_INCLUDES=-I$(HDMR_PATH)/include -I$(SOAP_PATH)/gsoap
MIDDLEWARE_LIBS=-L$(HDMR_PATH)/lib -L$(SOAP_PATH)/gsoap -lirobot_wrap -lirobot_app -lirobot_common -lPeer -lUdata -lpthread


# Change this to your paths
#IROBOTPATH = ../../irobot
IPATH = .
#IPATH += $(IROBOTPATH)/include

VPATH = .
VPATH += og
VPATH += utils

# Change this to whatever name you like
EXE = morph

static_LIB = libmorph_base.a
shared_LIB = libmorph_base.so

CXXFLAGS =-O2 -Wall -Wno-reorder  -mcpu=bf561 -DWITH_MOTION -DWIP -DTEST_CLIMBING
CFLAGS = -O2 -Wall -Wno-reorder -mcpu=bf561 -std=gnu99 -DWITH_MOTION -DWIP -DTEST_CLIMBING


CXXFLAGS += $(patsubst %, -I%, $(IPATH)) $(MIDDLEWARE_INCLUDES)
LDFLAGS = $(MIDDLEWARE_LIBS) -lpthread 
LDFLAGS_$(shared_LIB) = -shared -Wl $(MIDDLEWARE_LIBS)

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

