
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
CXXFLAGS     := -Wall -O2 -fPIC -std=c++20 -pthread -I $(includeprefix)

LDFLAGS      :=
LDLIBS       :=

# Optional libgpiod support (needed for Pi 5 and future models).
# Requires libgpiod-dev; disabled if not found.
LIBGPIOD_CFLAGS := $(shell pkg-config --cflags libgpiod 2>/dev/null)
ifneq ($(LIBGPIOD_CFLAGS),)
    CXXFLAGS += -DHAVE_LIBGPIOD $(LIBGPIOD_CFLAGS)
    $(info libgpiod found — Pi 5 GPIO support enabled)
else
    $(info libgpiod not found — Pi 5 GPIO support disabled (install libgpiod-dev to enable))
endif


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
