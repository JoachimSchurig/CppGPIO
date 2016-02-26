
LIBTOOL      := ar rucs
MKDIR        := mkdir -p
CP           := cp
CD           := cd
CHOWN	     := chown
CHMOD        := chmod
LN           := ln -sf
RM           := rm -f
RMDIR        := rm -rf
MAKE         := make

sourceprefix := ./
includeprefix:= ../include
headerprefix := ../include/cppgpio
SRCS         := $(shell find $(sourceprefix) -maxdepth 1 -name "*.cpp")
SRCS         := $(SRCS:./%=%)
HDRS         := $(shell find $(headerprefix) -maxdepth 1 -name "*.hpp")
HDRS         := $(HDRS:./%=%)
OBJS         := $(SRCS:.cpp=.o)

CXX          := g++
CXXFLAGS     := -Wall -O2 -fPIC -std=c++11 -pthread -I $(includeprefix)

ifneq ("$(wildcard /usr/include/linux/i2c-dev.h)","")
CXXFLAGS     += -DHAVE_I2C_HEADERS
endif

LDFLAGS :=
LDLIBS       :=


all: lib

.PHONY: all lib depend clean dist-clean install uninstall

lib: $(OBJS)

depend: .depend

.depend: $(SRCS)
	$(RM) ./.depend
	$(CXX) $(CXXFLAGS) -MM $^ >> ./.depend;

clean:
	$(RM) $(OBJS)

dist-clean: clean
	$(RM) *~ .depend

-include .depend
