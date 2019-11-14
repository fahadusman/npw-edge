/*
 * CommDataBuffer.h
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 *
 *  This class represents data buffers for transmission over radio
 *  link as well as over MQTT. Initially we have two types of data;
 *  	1. NPW Buffers
 *  	2. Periodic data
 */

#ifndef INCLUDE_COMMDATABUFFER_H_
#define INCLUDE_COMMDATABUFFER_H_
#include <cstdint>
#include <string>
#include "NpwBuffer.h"

enum BufferType {npwBuffer, periodicData};
class CommDataBuffer {
private:
	void * dataBuffer;
	double value;
	int length;
	uint64_t timestamp;
	BufferType type;
public:
	std::string serialize();
	CommDataBuffer(NpwBuffer * npwBufferPtr);
	CommDataBuffer();
	uint64_t getTimestamp(){
		return timestamp;
	}
};



#endif /* INCLUDE_COMMDATABUFFER_H_ */
