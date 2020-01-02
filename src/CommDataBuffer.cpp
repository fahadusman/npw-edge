/*
 * CommDataBuffer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#include "CommDataBuffer.h"

#include <iostream>
#include <glog/logging.h>

unsigned int CommDataBuffer::bufferCount = 0;

CommDataBuffer::CommDataBuffer(){
	timeStamp = 100;
	length = 0;
	bufferId = bufferCount++;
	sensorId = "";
	expiryTime = 0;
}

void CommDataBuffer::setExpiryTime(uint64_t et) {
    if (et > 0) {
        expiryTime = et;
    } else {
        LOG(WARNING) << "Invalid buffer expiry time: " << et;
    }
}

uint64_t CommDataBuffer::getExpiryTime() {
    return expiryTime;
}
//std::string CommDataBuffer::serializeJson(){
//	return "{\"payload\": \"dummy payload\"}";
//}



