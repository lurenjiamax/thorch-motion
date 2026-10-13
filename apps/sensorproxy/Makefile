# Use matching public headers from the corrected 0.4.4 + rate source.
CC ?= cc
SSC_INCLUDE ?= ../../motion-20261012/libssc-placement-study/libssc-placement-rate/src
CPPFLAGS += -I$(SSC_INCLUDE) $(shell pkg-config --cflags gio-2.0 gmodule-2.0)
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Werror
LDLIBS += $(shell pkg-config --libs gio-2.0 gmodule-2.0) -lm
all: thor-sensorproxy
thor-sensorproxy: thor-sensorproxy.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)
.PHONY: all
