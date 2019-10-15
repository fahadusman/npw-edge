#ifndef INCLUDE_SERIALPORT_H_
#define INCLUDE_SERIALPORT_H_

/*
 * serialPort.h
 *
 *  Created on: Oct 2, 2019
 *      Author: Fahad Usman
 */
//#define NDEBUG

#include <fcntl.h>
#include <termios.h>
#include <string>

const int kDefaultBaudRate = B115200;
const int kDefaultParity = 0;
const int kDefaultBlocking = 0;
const std::string kDefaultPortName = "/dev/ttyUSB0"; //"/dev/ttyM0";

class SerialPort {
protected:
	int device;
	int speed;
	int parity;
	int blocking;
	std::string portName;
    termios originalTtyAttributes;

	int setInterfaceAttributes();
	void openPort();
	void closePort();
public:
	SerialPort();
	SerialPort(const std::string portName, int spd, int par, int shouldBlock);
	~SerialPort();

	ssize_t readBuffer(unsigned char * buff, int len);
	ssize_t writeBuffer(const unsigned char * buff, int len);
};



#endif /* INCLUDE_SERIALPORT_H_ */
