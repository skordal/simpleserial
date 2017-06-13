// SimpleSerial - A simple serial port library for Java
// (c) Kristian Klomsten Skordal 2017 <kristian.skordal@wafflemail.net>
// Report bugs and issues on <https://github.com/skordal/simpleserial/issues>

package net.skordal.simpleserial;

public class SerialPortException extends Exception
{
	private static final long serialVersionUID = 27482L;

	public SerialPortException(String message)
	{
		super(message);
	}
}

