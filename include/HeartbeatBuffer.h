/*
 * HeartbeatBuffer.h
 *
 *  Created on: Mar 13, 2020
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_HEARTBEATBUFFER_H_
#define INCLUDE_HEARTBEATBUFFER_H_

#include "CommDataBuffer.h"

const int registerMapSize = 40;

class HeartbeatBuffer: public CommDataBuffer {
public:
    std::string serializeJson();
    unsigned char * serialize(int & length);
    bool deserialize(const unsigned char *, const int & length);

    HeartbeatBuffer(uint32_t devId);
    virtual ~HeartbeatBuffer();
private:
    uint32_t deviceId;
    uint32_t registerMap[registerMapSize];
};

#endif /* INCLUDE_HEARTBEATBUFFER_H_ */
