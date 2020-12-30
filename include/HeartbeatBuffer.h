/*
 * HeartbeatBuffer.h
 *
 *  Created on: Mar 13, 2020
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_HEARTBEATBUFFER_H_
#define INCLUDE_HEARTBEATBUFFER_H_

#include "CommDataBuffer.h"

#include <array>
#include <map>

const std::size_t registerMapSize = 50;

class HeartbeatBuffer: public CommDataBuffer {
public:
    std::string serializeJson();
    unsigned char * serialize(int & length);
    int deserialize(const unsigned char *, const int & length);

    HeartbeatBuffer(uint32_t devId,
            std::array<int32_t, registerMapSize> regMap,
            std::string devName);
    HeartbeatBuffer();
    virtual ~HeartbeatBuffer();
    static std::map<int, std::string> deviceNameMap;
    size_t getSerializedBuffLen() override;
private:
    uint32_t deviceId;
    int32_t registerMap[registerMapSize];
    std::string deviceName;
};

#endif /* INCLUDE_HEARTBEATBUFFER_H_ */
