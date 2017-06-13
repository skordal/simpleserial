// SimpleSerial - A simple serial port library for Java
// (c) Kristian Klomsten Skordal 2017 <kristian.skordal@wafflemail.net>
// Report bugs and issues on <https://github.com/skordal/simpleserial/issues>

package net.skordal.simpleserial;

public class NotASerialPortException extends SerialPortException
{
	public NotASerialPortException(String filename)
	{
		super("file \"" + filename + "\" is not a serial port");
	}
}

