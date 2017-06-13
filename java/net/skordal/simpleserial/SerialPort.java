// SimpleSerial - A simple serial port library for Java
// (c) Kristian Klomsten Skordal 2017 <kristian.skordal@wafflemail.net>
// Report bugs and issues on <https://github.com/skordal/simpleserial/issues>

package net.skordal.simpleserial;

import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;
import java.io.FileNotFoundException;

public class SerialPort
{
	public static final int PARITY_NONE = 0;
	public static final int PARITY_EVEN = 1;
	public static final int PARITY_ODD  = 2;

	private String filename;
	private int baudrate;
	private int parity;

	private int handle;
	private boolean opened = false;

	public SerialPort(String filename, int baudrate, int parity)
		throws SerialPortException, NotASerialPortException, FileNotFoundException, IOException
	{
		this.filename = filename;
		this.baudrate = baudrate;
		this.parity = parity;

		nativeOpen();
	}

	public String getFilename()
	{
		return filename;
	}

	public int getBaudrate()
	{
		return baudrate;
	}

	public boolean isOpen()
	{
		return opened;
	}

	public void close()
	{
		nativeClose();
	}

	@Override protected void finalize()
	{
		if(isOpen())
			close();
	}

	public InputStream getInputStream()
	{
		return new InputStream(){
			@Override public int read() throws IOException
			{
				return nativeRead();
			}
		};
	}

	public OutputStream getOutputStream()
	{
		return new OutputStream(){
			@Override public void write(int b) throws IOException
			{
				nativeWrite(new Integer(b).byteValue());
			}
		};
	}

	private native void nativeOpen() throws SerialPortException, NotASerialPortException, FileNotFoundException, IOException;
	private native void nativeClose();
	private native int nativeRead() throws IOException;
	private native void nativeWrite(byte b) throws IOException;

	static {
		System.loadLibrary("simpleserial");
	}
}

