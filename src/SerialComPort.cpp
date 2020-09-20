/*
 * serialPort.cpp
 *
 *  Created on: Oct 7, 2019
 *      Author: Fahad Usman
 */

#include "SerialComPort.h"

#include <glog/logging.h>
#include <iostream>
#include <string.h>

unsigned int serialPortFlags = O_RDWR | O_NOCTTY | O_SYNC;

SerialComPort::SerialComPort()
{
	speed = kDefaultBaudRate;
	parity = kDefaultParity;
	blocking = kDefaultBlocking;
	portName = kDefaultPortName;
	device = 0;
	openPort();
	setInterfaceAttributes();
}

SerialComPort::SerialComPort(const std::string pName, int spd, int par, int shouldBlock)
{
	LOG(INFO) << "SerialPort::SerialPort(" << pName << ", " << spd << ", " << par  << ", " << shouldBlock << ")";
	speed = spd;
	parity = par;
	blocking = shouldBlock;
	if (pName == ""){
		LOG(WARNING) << "no port name given, using default";
		portName = kDefaultPortName;
	}
	else{
		portName = pName;
	}

	openPort();
	setInterfaceAttributes();
}

SerialComPort::~SerialComPort(){
	closePort();
//	delete portName;
//	portName = NULL;
}

//takes preallocated buffer to read data from serial port
//returns the number of bytes read
ssize_t SerialComPort::readBuffer(unsigned char * buff, int len){
	if(device < 0){
	    LOG_EVERY_N(ERROR, 50) << "invalid device: " << device;
		return -1;
	}
	return read(device, buff, len);
	//TODO: Exception handling
}

ssize_t SerialComPort::writeBuffer(const unsigned char * buff, int len){
	if(device < 0){
	    LOG_EVERY_N(ERROR, 50) << "invalid device: " << device;
		return -1;
	}
	int res = write(device, buff, len);
    if (res < 0){
    	LOG(FATAL) << "write failed";
    }
	return res;
}

void SerialComPort::openPort(){
	LOG(INFO) << "Opening Serial Port" << portName;
	device = open(portName.c_str(), serialPortFlags);
	if (device < 0){
		LOG(ERROR) << "unable to open serialPort:" << portName;
	}
	else {
		LOG(INFO) << "successfully opened: " << portName << " FD: " << device;
		setInterfaceAttributes();
	}
}

void SerialComPort::closePort(){
	if (device >= 0){
	    if (tcsetattr (device, TCSANOW, &originalTtyAttributes) != 0)
	    {
	        LOG(ERROR) << "error setting interface attributes";
	    }
		close(device);
		device = -1;
		LOG(INFO) << "Closing serial port: " << device;
	}
	else {
		LOG(ERROR) << "invalid fd in closePort: " << device;
	}
}

int SerialComPort::setInterfaceAttributes(){
    termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (device, &tty) != 0)
    {
    	LOG(ERROR) << "error: " << errno << " from tcgetattr";
		return 0;
    }
    originalTtyAttributes = tty;
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;                 // disable break processing
    tty.c_lflag = 0;                        // no signaling chars, no echo,
    tty.c_oflag = 0;                        // no remapping, no delays

    tty.c_cc[VMIN]  = blocking ? 1 : 0;		// should read block or not
    tty.c_cc[VTIME] = 1;                    // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
                                            // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (device, TCSANOW, &tty) != 0)
    {
        LOG(ERROR) << "error setting interface attributes, errno:" << errno;
        return 0;
    }

	return 1;
}
