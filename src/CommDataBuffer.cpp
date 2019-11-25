/*
 * CommDataBuffer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#include <iostream>
#include "CommDataBuffer.h"

unsigned int CommDataBuffer::bufferCount = 0;

CommDataBuffer::CommDataBuffer(){
	timeStamp = 100;
	length = 0;
	bufferId = bufferCount++;
}

//std::string CommDataBuffer::serializeJson(){
//	return "{\"payload\": \"dummy payload\"}";
//}



