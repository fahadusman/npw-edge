/*
 * CommDataBuffer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#include <iostream>
#include "CommDataBuffer.h"
CommDataBuffer::CommDataBuffer(){
	dataBuffer = NULL;
	value = 0;
	timestamp = 100;
	length = 0;
	type = npwBuffer;
}

CommDataBuffer::CommDataBuffer(NpwBuffer * npwBufferPtr):CommDataBuffer(){
	std::cout << npwBufferPtr->getTimestamp();
}

std::string CommDataBuffer::serialize(){
	return "{\"payload\": \"dummy payload\"}";
}



