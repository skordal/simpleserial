// SimpleSerial - A simple serial port library for Java
// (c) Kristian Klomsten Skordal 2017 <kristian.skordal@wafflemail.net>
// Report bugs and issues on <https://github.com/skordal/simpleserial/issues>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <jni.h>

#include "net_skordal_simpleserial_SerialPort.h"

#define JAVA_SIGNATURE_STRING  "Ljava/lang/String;"
#define JAVA_SIGNATURE_INT     "I"
#define JAVA_SIGNATURE_BOOLEAN "Z"

static struct
{
	int     baudrate;
	speed_t speed;
} baudrateLookupTable[] = {
	{0, B0},
	{50, B50},
	{75, B75},
	{110, B110},
	{134, B134},
	{150, B150},
	{200, B200},
	{300, B300},
	{600, B600},
	{1200, B1200},
	{1800, B1800},
	{2400, B2400},
	{4800, B4800},
	{9600, B9600},
	{19200, B19200},
	{38400, B38400},
	{57600, B57600},
	{115200, B115200},
	{230400, B230400},
	{460800, B460800},
	{500000, B500000},
	{576000, B576000},
	{921600, B921600},
	{1000000, B1000000},
	{1152000, B1152000},
	{1500000, B1500000},
	{2000000, B2000000},
	{2500000, B2500000},
	{3000000, B3000000},
	{3500000, B3500000},
	{4000000, B4000000}
};

static void throwFileNotFound(JNIEnv * env, const char * filename)
{
	char errorMessageTemplate[] = "serial port file not found: ";
	char errorMessage[strlen(filename) + strlen(errorMessageTemplate) + 1];
	snprintf(errorMessage, sizeof(errorMessage), "%s%s", errorMessageTemplate, filename);

	jclass exception = (*env)->FindClass(env, "java/io/FileNotFoundException");
	(*env)->ThrowNew(env, exception, errorMessage);
}

static void throwIOException(JNIEnv * env, const char * message)
{
	jclass exception = (*env)->FindClass(env, "java/io/IOException");
	(*env)->ThrowNew(env, exception, message);
}

static bool configurePort(JNIEnv * env, jobject object, int handle, int baudrate, int parity)
{
	jclass class = (*env)->GetObjectClass(env, object);
	jfieldID parityNoneField = (*env)->GetStaticFieldID(env, class, "PARITY_NONE", JAVA_SIGNATURE_INT);
	jfieldID parityEvenField = (*env)->GetStaticFieldID(env, class, "PARITY_EVEN", JAVA_SIGNATURE_INT);
	jfieldID parityOddField = (*env)->GetStaticFieldID(env, class, "PARITY_ODD", JAVA_SIGNATURE_INT);

	const int parityNoneValue = (*env)->GetStaticIntField(env, object, parityNoneField);
	const int parityEvenValue = (*env)->GetStaticIntField(env, object, parityEvenField);
	const int parityOddValue = (*env)->GetStaticIntField(env, object, parityOddField);

	struct termios config;
	int status = tcgetattr(handle, &config);
	if(status != 0)
	{
		jclass exception = (*env)->FindClass(env, "net/skordal/simpleserial/SerialPortException");
		(*env)->ThrowNew(env, exception, "failed to get serial port attributes with tcgetattr()");
		return false;
	}

	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag = 0;
	config.c_lflag = (ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;

	speed_t speed;
	bool speedFound = false;
	for(int i = 0; i < sizeof(baudrateLookupTable); ++i)
	{
		if(baudrateLookupTable[i].baudrate == baudrate)
		{
			speedFound = true;
			speed = baudrateLookupTable[i].speed;
			break;
		}
	}

	if(!speedFound)
	{
		jclass exception = (*env)->FindClass(env, "net/skordal/simpleserial/SerialPortException");
		(*env)->ThrowNew(env, exception, "invalid baudrate specified");
		return false;
	}

	status = cfsetispeed(&config, speed);
	if(status != 0)
	{
		jclass exception = (*env)->FindClass(env, "net/skordal/simpleserial/SerialPortException");
		(*env)->ThrowNew(env, exception, "failed to set serial port input speed with cfsetispeed()");
		return false;
	}

	status = cfsetospeed(&config, speed);
	if(status != 0)
	{
		jclass exception = (*env)->FindClass(env, "net/skordal/simpleserial/SerialPortException");
		(*env)->ThrowNew(env, exception, "failed to set serial port output speed with cfsetospeed()");
		return false;
	}

	return true;
}

JNIEXPORT void JNICALL Java_net_skordal_simpleserial_SerialPort_nativeOpen(JNIEnv * env, jobject object)
{
	jclass class = (*env)->GetObjectClass(env, object);

	jfieldID filenameField = (*env)->GetFieldID(env, class, "filename", JAVA_SIGNATURE_STRING);
	jfieldID handleField = (*env)->GetFieldID(env, class, "handle", JAVA_SIGNATURE_INT);
	jfieldID openedField = (*env)->GetFieldID(env, class, "opened", JAVA_SIGNATURE_BOOLEAN);
	jfieldID baudrateField = (*env)->GetFieldID(env, class, "baudrate", JAVA_SIGNATURE_INT);
	jfieldID parityField = (*env)->GetFieldID(env, class, "parity", JAVA_SIGNATURE_INT);

	jstring filenameString = (*env)->GetObjectField(env, object, filenameField);
	const char * filename = (*env)->GetStringUTFChars(env, filenameString, 0);

	int handle = open(filename, O_RDWR | O_NOCTTY);
	if(handle == -1)
	{
		int error = errno;
		switch(error)
		{
			case ENOENT:
				throwFileNotFound(env, filename);
				break;
			default:
				throwIOException(env, "could not open serial port");
				break;
		}

		goto _cleanup;
	}

	if(!isatty(handle))
	{
		jclass exception = (*env)->FindClass(env, "net/skordal/serialport/NotASerialPortException");
		(*env)->ThrowNew(env, exception, filename);
		close(handle);
		goto _cleanup;
	}

	int baudrate = (*env)->GetIntField(env, object, baudrateField);
	int parity = (*env)->GetIntField(env, object, parityField);
	bool configured = configurePort(env, object, handle, baudrate, parity);
	if(!configured)
	{
		close(handle);
		goto _cleanup;
	}

	// Set the handle and opened fields in the class:
	(*env)->SetIntField(env, object, handleField, handle);
	(*env)->SetBooleanField(env, object, openedField, JNI_TRUE);

_cleanup:
	(*env)->ReleaseStringUTFChars(env, filenameString, filename);
}

JNIEXPORT void JNICALL Java_net_skordal_simpleserial_SerialPort_nativeClose(JNIEnv * env, jobject object)
{
	jclass class = (*env)->GetObjectClass(env, object);

	jfieldID handleField = (*env)->GetFieldID(env, class, "handle", JAVA_SIGNATURE_INT);

	int handle = (*env)->GetIntField(env, object, handleField);
	close(handle);
}

JNIEXPORT jint JNICALL Java_net_skordal_simpleserial_SerialPort_nativeRead(JNIEnv * env, jobject object)
{
	jclass class = (*env)->GetObjectClass(env, object);
	jfieldID handleField = (*env)->GetFieldID(env, class, "handle", JAVA_SIGNATURE_INT);

	int handle = (*env)->GetIntField(env, object, handleField);

	char byte;
	int status = read(handle, &byte, 1);
	if(status == -1)
	{
		int error = errno;
		jclass exception = (*env)->FindClass(env, "java/io/IOException");
		(*env)->ThrowNew(env, exception, strerror(error));
		return -1;
	}

	return (jint) byte;
}

JNIEXPORT void JNICALL Java_net_skordal_simpleserial_SerialPort_nativeWrite(JNIEnv * env, jobject object, jbyte byte)
{
	jclass class = (*env)->GetObjectClass(env, object);
	jfieldID handleField = (*env)->GetFieldID(env, class, "handle", JAVA_SIGNATURE_INT);

	int handle = (*env)->GetIntField(env, object, handleField);

	char c = byte;
	int status = write(handle, &c, 1);
	if(status == -1)
	{
		int error = errno;
		jclass exception = (*env)->FindClass(env, "java/io/IOException");
		(*env)->ThrowNew(env, exception, strerror(error));
	}
}

