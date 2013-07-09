# makefile configuration
SRC	= main.cc
SRC	+= global.cc

TARGET_PROG = logtest

OBJDIR = build_$(TARGET_PROG)
OBJECTS = $(SRC:%.cc=$(OBJDIR)/%.o)

BINDIR = ./


IPATH = .
IPATH += ../
IPATH += ../../

VPATH = .
VPATH += ../

CC = g++
CFLAGS  =  -Wall -g -D_GNU_SOURCE
#CFLAGS += -DLAPTOP
CFLAGS += ${patsubst %,-I%,${IPATH}}


LIBS = 

.Phony: clean

all: mkobjectsdir $(TARGET_PROG) 

mkobjectsdir:
	mkdir -p $(OBJDIR)

$(TARGET_PROG): ${OBJECTS}
	${CC} ${CFLAGS} -o  ${BINDIR}/$@ $^ ${LIBS}

$(OBJDIR)/%.o: %.cc
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	${RM} -rf $(OBJDIR) ${BINDIR}/$(TARGET_PROG)
