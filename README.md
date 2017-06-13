# SimpleSerial

SimpleSerial is a simple serial port library for Java. It currently only supports
Linux, although you might get it working on other Unix systems.

## Building

To build, run `ant` in the toplevel directory. This will compile the Java sources
and automatically run `make` to compile the native components in the native/
directory.

## Using the Library

To use the library, you need the `simpleserial.jar` file as well as the `simpleserial.so`
file which is loaded by the Java runtime to provide the necessary interface to the
operating system's serial port functionality.

