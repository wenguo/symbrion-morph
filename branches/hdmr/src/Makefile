SRC = robot_KIT.cc robot_AW.cc robot_SCOUT.cc main.cc

CC = bfin-linux-uclibc-gcc
CXX = bfin-linux-uclibc-g++ 
AR = bfin-linux-uclibc-ar
OBJDUMP = bfin-linux-uclibc-objdump
STRIP = bfin-linux-uclibc-strip

# Change this to your paths
MORPHBASEPATH = ./base
IPATH = .
IPATH += $(MORPHBASEPATH)

VPATH = .

MIDDLEWARE_PATH_ROOT=$(HOME)

HDMR_PATH=$(MIDDLEWARE_PATH_ROOT)/hdmrplus_install
SOAP_PATH=$(MIDDLEWARE_PATH_ROOT)/gsoap-2.8

MIDDLEWARE_INCLUDES=-I$(HDMR_PATH)/include -I$(SOAP_PATH)/gsoap
MIDDLEWARE_LIBS=-L$(HDMR_PATH)/lib -L$(SOAP_PATH)/gsoap -lirobot_app -lirobot_common -lPeer -lUdata -lpthread



# Change this to whatever name you like
EXE = morph

CXXFLAGS =-O2 -Wall -Wno-reorder  -mcpu=bf561
CFLAGS = -O2 -Wall -Wno-reorder -mcpu=bf561 -std=gnu99

CXXFLAGS += $(patsubst %, -I%, $(IPATH)) $(MIDDLEWARE_INCLUDES)
LDFLAGS = $(MIDDLEWARE_LIBS) -L$(MORPHBASEPATH)/lib -lpthread -lmorph_base

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

objects/%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
objects/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
	
clean:
	rm -f bin/$(EXE) objects/*.o objects/*.lst
	@mkdir -p objects bin
	rm -rf objects bin

