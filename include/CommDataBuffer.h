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

class CommDataBuffer {
protected:
    unsigned int length;
    uint64_t timeStamp;
public:
    virtual std::string serializeJson() = 0;
    virtual void * serialize(int & length) = 0;
    CommDataBuffer();
    uint64_t getTimestamp() {
        return timeStamp;
    }
    virtual ~CommDataBuffer() {
    }
};



#endif /* INCLUDE_COMMDATABUFFER_H_ */
