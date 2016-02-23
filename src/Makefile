
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

sourcepref   := ./
includepref  := ../include
headerpref   := ../include/cppgpio
SRCS         := $(shell find $(sourcepref) -maxdepth 1 -name "*.cpp")
SRCS         := $(SRCS:./%=%)
HDRS         := $(shell find $(headerpref) -maxdepth 1 -name "*.hpp")
HDRS         := $(HDRS:./%=%)
OBJS         := $(SRCS:.cpp=.o)

CXX          := g++
CXXFLAGS     := -Wall -O2 -fPIC -std=gnu++14 -pthread -I $(includepref)
LDFLAGS      :=
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
