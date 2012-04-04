#!/bin/make

# ----------------------------------------------------------------------------------------------------------------
# Project:	Symbrion + Replicator
# File:		Makefile
# Authors:	Florian Schlachter, IPVS - Universität Stuttgart
#			Christopher Schwarzer, Universität Tübingen
#			Benjamin Girault, IPVS - Universität Stuttgart
# 			Anne van Rossum, Almende
# ----------------------------------------------------------------------------------------------------------------

# Optionally include local.mk, for example set PATH+=:/opt/uClinux/bfin-linux-uclibc/bin/
# in case you installed the blackfin toolchain via apt-get install blackfin-toolchain-linux-uclibc
-include local.mk

SRC = SPIStreamMaster.cpp SPIStream.cpp SPIController.cpp Ethernet.cpp IRComm.cpp Microphone.cpp
SRC += bytequeue.c gpio.cpp lolmsg.c Memmo.cpp IRobot.cpp streamcoder.c RobotRNG.cpp

ifeq ($(MULTI_CONTROLLER),true)
SRC += ActionSelection.cpp ConfigFile.cpp Controllers.cpp OperatingConditions.cpp
endif

APPSRC = TestMain.cpp

ROOTPATH = ../controller/irobotlib

# Default flags
# Anne: please, consider using -pedantic, because it is often not normal C, e.g. using b00000000 is a C extension
# and does not work with my compiler
CXXFLAGS = -O2 -Wall
CFLAGS = -O2 -Wall -std=gnu99

# Define cross-compile target
TARGET=bfin-linux-uclibc-

# Just use normal compiler, uncomment if you want to cross-compile
# This will add a -DRUNONPC flag to gcc or g++ which can subsequently be
# used in the code to distinguish between code for on the robot and on the PC
ifeq ($(RUNONPC),true)
TARGET=
CFLAGS += -DRUNONPC
CXXFLAGS += -DRUNONPC
endif

ifeq ($(MULTI_CONTROLLER),true)
CFLAGS += -DMULTI_CONTROLLER
CXXFLAGS += -DMULTI_CONTROLLER
endif


CC = $(TARGET)gcc
CXX = $(TARGET)g++ 
AR = $(TARGET)ar
OBJDUMP = $(TARGET)objdump
SIZE = $(TARGET)size
STRIP = $(TARGET)strip

ifneq ($(CC),gcc)
	CXXFLAGS += -mcpu=bf561
	CFLAGS += -mcpu=bf561 
endif

LIB = libirobot.a
shared_LIB= libirobot.so
TESTAPP = robotest

VPATH = .
VPATH += comm
ifeq ($(MULTI_CONTROLLER),true)
VPATH += action
endif

IPATH = .

CXXFLAGS += $(patsubst %, -I%, $(IPATH))
LDFLAGS = -L./lib -lpthread -lirobot
LDFLAGS_$(shared_LIB) = -shared -Wl -L./lib

TOBJECTS = $(SRC:%.cpp=objects/%.o)
OBJECTS = $(TOBJECTS:%.c=objects/%.o)

APPOBJECTS = $(APPSRC:%.cpp=objects/%.o)

all: objects lib bin include lib/$(LIB) lib/$(shared_LIB) includes bin/$(TESTAPP) strip

objects:
	mkdir objects

lib:
	mkdir lib

include:
	mkdir include
	mkdir include/comm
ifeq ($(MULTI_CONTROLLER),true)
	mkdir include/action
endif

bin:
	mkdir bin

includes: include
	cp  $(wildcard *.h) ./include
	cp  $(wildcard comm/*.h) ./include/comm
ifeq ($(MULTI_CONTROLLER),true)
	cp -u $(wildcard action/*.h) ./include/action
endif

lib/$(LIB): $(OBJECTS)
	$(AR) rcu $@ $^
	$(OBJDUMP) -hS $@ > objects/$(LIB).lst

lib/$(shared_LIB): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS_$(shared_LIB))
	$(OBJDUMP) -hS $@ > objects/$(shared_LIB).lst


bin/$(TESTAPP): $(APPOBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)
	$(OBJDUMP) -hS $@ > objects/$(TESTAPP).lst
	$(SIZE) -B $@

objects/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

objects/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

strip:
	@$(STRIP) bin/$(TESTAPP)

clean_install:
ifeq ($(MULTI_CONTROLLER),true)
	rm -f $(ROOTPATH)/include/action/*.h
endif
	rm -f $(ROOTPATH)/include/comm/*.h
	@rm -f $(ROOTPATH)/include/*.h

install: lib/$(LIB) includes clean_install
	cp -R include/* $(ROOTPATH)/include/
	cp lib/$(LIB) $(ROOTPATH)/lib/
	cp lib/$(shared_LIB) $(ROOTPATH)/lib/

# Apologies in advance! I do not want to have "rm -rf" things in my script (rm -f is fine).
# Suppose you move your Makefile somewhere else and it accidently runs the clean target...
# However rmdir has this obnoxious warning if the directory doesn't exist
# So, easiest is to just do mkdir before running rmdir :-)
# Needs to be updated of course when you add (sub)directories
clean:
ifeq ($(MULTI_CONTROLLER),true)
	rm -f include/action/*.h
endif	
	rm -f lib/$(LIB) lib/$(shared_LIB) bin/$(TESTAPP) objects/*.o objects/*.lst include/*.h include/comm/*.h
	@mkdir -p objects include include/comm lib bin
ifeq ($(MULTI_CONTROLLER),true)
	@mkdir -p include/action
	rmdir include/action 
endif	
	rmdir objects include/comm include lib

help:
	@echo "Set environmental variable RUNONPC=true or RUNONPC=false (or nothing) beforehand"
	@echo "You can also set this variable in local.mk"
	@echo "If RUNONPC=true is used, CC=gcc, else CC=bfin-linux-uclibc-gcc"
