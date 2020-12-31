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

const size_t sensorIdLen = 10;

enum BufferType {
    buffTypePeriodicValue,
    buffTypeNpwBuffer,
    buffTypeHeartBeat,
    buffTypeMultiple
};

class CommDataBuffer {
protected:
    static uint32_t bufferCount;
    unsigned int length;
    uint64_t timeStamp;
    uint16_t bufferId;
    char sensorId[sensorIdLen];
    uint64_t expiryTime;
public:
    virtual std::string serializeJson() = 0;
    virtual unsigned char * serialize(int & length) = 0;
    virtual int deserialize(const unsigned char *, const int & length) = 0;
    CommDataBuffer();
    uint64_t getTimestamp() {
        return timeStamp;
    }
    uint16_t getBufferId() {
        return bufferId;
    }
    void setExpiryTime(uint64_t et);
    uint64_t getExpiryTime();
    virtual size_t getSerializedBuffLen() = 0;
    virtual ~CommDataBuffer() {
    }
};



#endif /* INCLUDE_COMMDATABUFFER_H_ */
