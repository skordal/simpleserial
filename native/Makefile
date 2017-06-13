# SimpleSerial - A simple serial port library for Java
# (c) Kristian Klomsten Skordal 2017 <kristian.skordal@wafflemail.net>
# Report bugs and issues on <https://github.com/skordal/simpleserial/issues>

.PHONY: all clean

JDK_HOME ?= /usr/java/latest

CC ?= gcc
CFLAGS += \
	-I$(JDK_HOME)/include \
	-I$(JDK_HOME)/include/linux \
	-fPIC

SOURCE_FILES := \
	net_skordal_simpleserial_SerialPort.c
OBJECT_FILES := $(SOURCE_FILES:.c=.o)

all: $(OBJECT_FILES)
	test -d ../lib || mkdir ../lib
	$(CC) $(CFLAGS) -o ../lib/libsimpleserial.so -shared $(OBJECT_FILES)

clean:
	-$(RM) $(OBJECT_FILES)
	-$(RM) ../lib/libsimpleserial.so

net_skordal_simpleserial_SerialPort.o: net_skordal_simpleserial_SerialPort.c net_skordal_simpleserial_SerialPort.h
	$(CC) -c $(CFLAGS) -o $@ $<

