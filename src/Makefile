SRC = robot_KIT.cc robot_AW.cc robot_SCOUT.cc main.cc

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip
SIZE = bfin-linux-uclibc-size

# Change this to your paths
IROBOTPATH = ../irobot
MORPHBASEPATH = ./base
IPATH = .
IPATH += ../cmvision
IPATH += $(IROBOTPATH)/include
IPATH += $(MORPHBASEPATH)

VPATH = .

# Change this to whatever name you like
EXE = morph

CXXFLAGS =-O2 -Wall -Wno-reorder  -mcpu=bf561
CFLAGS = -O2 -Wall -Wno-reorder -mcpu=bf561 -std=gnu99

CXXFLAGS += $(patsubst %, -I%, $(IPATH))
LDFLAGS = -L$(IROBOTPATH)/lib -L$(MORPHBASEPATH)/lib -lirobot -lpthread -lmorph_base

ifeq ($(MULTI_CONTROLLER),true)
CFLAGS += -DMULTI_CONTROLLER
CXXFLAGS += -DMULTI_CONTROLLER
LDFLAGS += -lrt
endif

TOBJECTS = $(SRC:%.cc=objects/%.o)
OBJECTS = $(TOBJECTS:%.c=objects/%.o)

all: objects bin  bin/$(EXE)

objects:
	mkdir objects

bin:
	mkdir bin

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
	rm -f bin/$(EXE) objects/*.o objects/*.lst
	@mkdir -p objects bin
	rm -rf objects bin

