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

enum BufferType {
    buffTypePeriodicValue,
    buffTypeNpwBuffer,
    buffTypeHeartBeat
};

class CommDataBuffer {
protected:
    static uint32_t bufferCount;
    unsigned int length;
    uint64_t timeStamp;
    std::string sensorId;
    uint16_t bufferId;
    uint64_t expiryTime;
public:
    virtual std::string serializeJson() = 0;
    virtual unsigned char * serialize(int & length) = 0;
    virtual bool deserialize(const unsigned char *, const int & length) = 0;
    CommDataBuffer();
    uint64_t getTimestamp() {
        return timeStamp;
    }
    uint32_t getBufferId() {
        return bufferId;
    }
    void setExpiryTime(uint64_t et);
    uint64_t getExpiryTime();
    virtual ~CommDataBuffer() {
    }
};



#endif /* INCLUDE_COMMDATABUFFER_H_ */
