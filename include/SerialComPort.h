#ifndef INCLUDE_SERIALCOMPORT_H_
#define INCLUDE_SERIALCOMPORT_H_

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

class SerialComPort {
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
	SerialComPort();
	SerialComPort(const std::string portName, int spd, int par, int shouldBlock);
	~SerialComPort();

	ssize_t readBuffer(unsigned char * buff, int len);
	ssize_t writeBuffer(const unsigned char * buff, int len);
};



#endif /* INCLUDE_SERIALCOMPORT_H_ */
