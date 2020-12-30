/*
 * PeriodicValue.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: Fahad Usman
 */

#include "PeriodicValue.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

std::string PeriodicValue::serializeJson() {
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();
    writer.Key(sensorId);
    writer.Double(sensorValue);
    writer.Key((std::string("t_") + sensorId).c_str());
    writer.Uint64(timeStamp);
    writer.Key((std::string("q_") + sensorId).c_str());
    writer.Int(sensorStatus);
    writer.EndObject();

    return s.GetString();
}

size_t PeriodicValue::getSerializedBuffLen(void) {
    return (1 + sizeof(bufferId) + sizeof(sensorValue) + sizeof(timeStamp)
            + sizeof(sensorStatus) + sensorIdLen);
}

/*
 * Serialize and object into a binary buffer.
 * Returns a dynamically allocated buffer with serialized object,
 * and sets the reference of length.
 * In case of failure returns nullptr and sets length to 0.
 *
 * Byte 0: Buffer Type
 * Byte 1-2 (4b): BufferId
 * Byte 3-10 (8b): Sensor value
 * 10-18 (8b): Timestamp
 * 4b: Status
 * 10b: Sensor Id
 */
unsigned char * PeriodicValue::serialize(int & length) {
    unsigned char * serialBuffer = nullptr;
    try {
        length = getSerializedBuffLen();

        serialBuffer = new (std::nothrow) unsigned char [length];
        if (serialBuffer == nullptr) {
            LOG(ERROR) << "Unable to allocate memory for serialBuffer";
            length = 0;
            return serialBuffer;
        }
        unsigned int i = 0;

        serialBuffer[i++] = (unsigned char)buffTypePeriodicValue; //buffer type

        std::memcpy(serialBuffer+i, &(bufferId), sizeof(bufferId));
        i += sizeof(bufferId);

        std::memcpy(serialBuffer+i, &(sensorValue), sizeof(sensorValue));
        i += sizeof(sensorValue);

        std::memcpy(serialBuffer+i, &(timeStamp), sizeof(timeStamp));
        i+= sizeof(timeStamp);

        std::memcpy(serialBuffer+i, &(sensorStatus), sizeof(sensorStatus));
        i+= sizeof(sensorStatus);

        std::memcpy(serialBuffer+i, sensorId, sensorIdLen);

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
    return serialBuffer;
}

/*
 * Takes a binary serial buffer and uses it to populate the object this
 * function is called upon.
 * Returns true/false for success/failure respectively.
 */
int PeriodicValue::deserialize(const unsigned char * serialBuff, const int & len) {
    int expectedLength = getSerializedBuffLen();

    if (len < expectedLength) {
        LOG(WARNING) << "Length of serialized buffer for periodic value is too short: " << len;
        return 0;
    }
    unsigned int i = 1; //first (0th) byte is buffer-type, we don't need that here

    std::memcpy(&(bufferId), serialBuff+i, sizeof(bufferId));
    i += sizeof(bufferId);

    std::memcpy(&(sensorValue), serialBuff+i, sizeof(sensorValue));
    i += sizeof(sensorValue);

    std::memcpy(&(timeStamp), serialBuff+i, sizeof(timeStamp));
    i += sizeof(timeStamp);

    std::memcpy(&(sensorStatus), serialBuff+i, sizeof(sensorStatus));
    i += sizeof(sensorStatus);

    std::memcpy(sensorId, serialBuff + i, sensorIdLen);
    i += sensorIdLen;

    return i;
}
