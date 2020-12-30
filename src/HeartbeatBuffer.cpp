/*
 * HeartbeatBuffer.cpp
 *
 *  Created on: Mar 13, 2020
 *      Author: Fahad Usman
 */

#include "HeartbeatBuffer.h"

#include <cstring>
#include <chrono>
#include "glog/logging.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

std::string HeartbeatBuffer::serializeJson() {
    rapidjson::StringBuffer hbStringBuffer;
    rapidjson::Writer<rapidjson::StringBuffer> hbJsonWriter(hbStringBuffer);

    hbJsonWriter.StartObject();

    hbJsonWriter.Key(("t_hb_" + deviceName).c_str());
    hbJsonWriter.Uint64(timeStamp);
    hbJsonWriter.Key(("registerMap_" + deviceName).c_str());
    hbJsonWriter.StartArray();
    for (unsigned int i = 0; i < registerMapSize; i++) {
        hbJsonWriter.Int64(registerMap[i]);
    }
    hbJsonWriter.EndArray();
    hbJsonWriter.EndObject();

    return hbStringBuffer.GetString();
}

unsigned char * HeartbeatBuffer::serialize(int & length) {
    unsigned char * serialBuffer = nullptr;
    try {
        length = 1/*buffer type*/+ sizeof(bufferId) + sizeof(deviceId)
                + sizeof(timeStamp) + sizeof(registerMap);

        serialBuffer = new (std::nothrow) unsigned char [length];
        if (serialBuffer == nullptr) {
            LOG(ERROR) << "Unable to allocate memory for serialBuffer";
            length = 0;
            return serialBuffer;
        }
        unsigned int i = 0;
        DLOG(INFO) << "Allocated buffer for heartbeat of length: " << length;

        serialBuffer[i++] = (unsigned char)buffTypeHeartBeat; //buffer type

        std::memcpy(serialBuffer+i, &(bufferId), sizeof(bufferId));
        i += sizeof(bufferId);

        std::memcpy(serialBuffer+i, &(timeStamp), sizeof(timeStamp));
        i += sizeof(timeStamp);

        std::memcpy(serialBuffer+i, registerMap, sizeof(registerMap));
        i += sizeof(registerMap);
        //TODO: Add device stats and sensor and network health

        std::memcpy(serialBuffer+i, &(deviceId), sizeof(deviceId));
        i += sizeof(deviceId);

        return serialBuffer;
    }
    catch (const std::exception & e) {
        LOG(ERROR) << "Exception in serialize: " << e.what();
    }
    if (serialBuffer != nullptr) {
        delete serialBuffer;
        serialBuffer = nullptr;
    }
    length = 0;
    return serialBuffer;}

size_t HeartbeatBuffer::getSerializedBuffLen() {
    return 1/*buffer type*/+ sizeof(bufferId) + sizeof(deviceId)
                    + sizeof(timeStamp) + sizeof(registerMap);
}

int HeartbeatBuffer::deserialize(const unsigned char * hbBuffer,
        const int & length) {
    int expectedLength = getSerializedBuffLen();

    if (length < expectedLength) {
        LOG(WARNING)
                << "Length of serialized heartbeat buffer is not correct: "
                << length << "< Expected: " << expectedLength;
        return 0;
    }

    if (buffTypeHeartBeat != (BufferType)(hbBuffer[0])){
        LOG(WARNING)
                << "HeartbeatBuffer::deserialize(), incorrect buffer type"
                << (unsigned int) hbBuffer[0];
        return 0;
    }

    unsigned int i = 1; //first (0th) byte is buffer-type

    std::memcpy(&(bufferId), hbBuffer+i, sizeof(bufferId));
    i+= sizeof(bufferId);

    std::memcpy(&(timeStamp), hbBuffer+i, sizeof(timeStamp));
    i += sizeof(timeStamp);

    std::memcpy(registerMap, hbBuffer+i, sizeof(registerMap));
    i += sizeof(registerMap);

    std::memcpy(&(deviceId), hbBuffer+i, sizeof(deviceId));
    i += sizeof(deviceId);

    if (deviceNameMap.find(deviceId) != deviceNameMap.end()) {
        deviceName = deviceNameMap[deviceId];
    } else {
        LOG(WARNING) << "Failed to look up device name in map, deviceId: "
                << deviceId;
        deviceName = "InvalidDeviceName";
    }

    return i;
}

HeartbeatBuffer::HeartbeatBuffer(uint32_t devId,
        std::array<int32_t, registerMapSize> regMap,
        std::string devName) :
        deviceId(devId), deviceName(devName) {
    for (unsigned int i = 0; i < registerMapSize; i++) {
        registerMap[i] = regMap[i];
    }
    timeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}
HeartbeatBuffer::HeartbeatBuffer() {
    deviceId = 0;
    timeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    for (unsigned int i = 0; i < registerMapSize; i++) {
        registerMap[i] = 0;
    }
}

HeartbeatBuffer::~HeartbeatBuffer() {
}

