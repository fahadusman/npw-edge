/*
 * CommDataBuffer.cpp
 *
 *  Created on: Nov 5, 2019
 *      Author: Fahad Usman
 */

#include "CommDataBuffer.h"

#include <iostream>
#include <chrono>
#include <glog/logging.h>

unsigned int CommDataBuffer::bufferCount = 0;

CommDataBuffer::CommDataBuffer(){
	timeStamp = std::chrono::duration_cast<
            std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();;
	length = 0;
    bufferId = (bufferCount++)%65535; //bufferId is sent as 16-bit number is ack message
	sensorId[0] = 0;
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



