/*
 * NpwBuffer.cpp
 *
 *  Created on: Nov 13, 2019
 *      Author: Fahad Usman
 */

#include "NpwBuffer.h"
#include <ctime>
#include <chrono>

#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

const unsigned int kDefByteArrayLength = kDefNpwBufferLength*2 + kHdrLen;   //TODO: Make this dynamic
NpwBuffer::NpwBuffer() :
        readingList { } {
    timeStamp = 0;
}

NpwBuffer::NpwBuffer(uint64_t ts) :
        readingList { } {
    timeStamp = ts;
    LOG(INFO) << "new NPW Buffer, timeStamp: " << timeStamp << ", id: "
            << bufferId;
}

void NpwBuffer::insertAt(const unsigned int position, readingType value) {
    if (position >= kDefNpwBufferLength) {
        LOG(FATAL) << "array index out of bound. index=" << position;
    }
    readingList[position] = value;
}

inline unsigned char decToBcd(const unsigned char dec) {
    return ((dec % 100) / 10) * 16 + dec % 10;
}

/* Allocates a new NPW byte array buffer populates it according to the following rules;
 *
 * Byte                Value
 * Byte 0              Year 00..99 (00=2000, ..., 89=2089, 90=1990, 91=1991, ..., 99=1999)
 * Byte 1              Month 1..12
 * Byte 2              Day 1..31
 * Byte 3              Hour 0..23
 * Byte 4              Minute 0..59
 * Byte 5              Second 0..59
 * Byte 6              First two digits of the millisecond (00..99)
 * Byte 7              In the upper 4 bit: last digit of the millisecond 1..9
 *                     In the lower 4 bit: day of the week 1..7 (1=Sunday)
 * Byte 8              High byte of first 2-byte pressure value
 * Byte 9              Low byte of first 2-byte pressure value
 * Byte 10 …           More 2-byte values similar to bytes 8 and 9
 *
 */
unsigned char * NpwBuffer::createByteArray() {
    unsigned char * byteArray =
            new (std::nothrow) unsigned char[kDefByteArrayLength](); //2 bytes value, plus the header
    if (byteArray == nullptr) {
        LOG(ERROR) << "Unable to allocate memory for byteArray";
        return byteArray;
    }

    const auto durationSinceEpoch = std::chrono::milliseconds(timeStamp);
    const std::chrono::time_point<std::chrono::system_clock> tp_after_duration(
            durationSinceEpoch);
    time_t time_after_duration = std::chrono::system_clock::to_time_t(
            tp_after_duration);

    std::tm* npwTime = std::localtime(&time_after_duration);

    byteArray[0] = decToBcd(npwTime->tm_year % 100);
    byteArray[1] = decToBcd(npwTime->tm_mon + 1);
    byteArray[2] = decToBcd(npwTime->tm_mday);
    byteArray[3] = decToBcd(npwTime->tm_hour);
    byteArray[4] = decToBcd(npwTime->tm_min);
    byteArray[5] = decToBcd(npwTime->tm_sec);
    byteArray[6] = decToBcd((timeStamp % 1000) / 10); //First two digits of the millisecond (00..99)
    byteArray[7] = decToBcd((timeStamp % 10) * 10 + npwTime->tm_wday + 1);

    for (unsigned int i = 0; i < kDefNpwBufferLength; i++) {
        int currentVal = readingList[i];
        byteArray[2 * i + kHdrLen] = currentVal >> 8;
        byteArray[2 * i + kHdrLen + 1] = currentVal & 0xFF;
    }
    return byteArray;
}

/* Serialize NPW buffer into JSON object. This object would be published
 * on MQTT for Kepware to receive and serve it on OPC-UA as a byte array.
 */
std::string NpwBuffer::serializeJson() {
    const unsigned char * byteArray = createByteArray();
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);
    writer.StartObject();
    writer.Key("array");
    writer.StartArray();
    for (unsigned int i = 0; i < kDefByteArrayLength; i++)
        writer.Uint(byteArray[i]);
    writer.EndArray();
    writer.EndObject();

    delete byteArray;
    return s.GetString();
}

unsigned char * NpwBuffer::serialize(int & length) {
    unsigned char * serialBuffer = nullptr;
    try {
        length = 1/*buffer type*/+ sizeof(bufferId) + kDefByteArrayLength
                + sizeof(timeStamp) + sensorId.length() + 1/*null char for sensorId*/;

        serialBuffer = new (std::nothrow) unsigned char [length];
        if (serialBuffer == nullptr) {
            LOG(ERROR) << "Unable to allocate memory for serialBuffer";
            length = 0;
            return serialBuffer;
        }
        unsigned int i = 0;

        serialBuffer[i++] = (unsigned char)buffTypeNpwBuffer; //buffer type

        std::memcpy(serialBuffer+i, &(bufferId), sizeof(bufferId));
        i += sizeof(bufferId);

        unsigned char * byteArray = createByteArray();
        std::memcpy(serialBuffer+i, byteArray, kDefByteArrayLength);
        i += kDefByteArrayLength;
        delete byteArray;

        std::memcpy(serialBuffer+i, &(timeStamp), sizeof(timeStamp));
        i+= sizeof(timeStamp);

        std::memcpy(serialBuffer+i, sensorId.c_str(), sensorId.length());
        serialBuffer[i+sensorId.length()] = '\0';
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

bool NpwBuffer::deserialize(const unsigned char * serialBuff, const int & len) {
    if ((unsigned int) len < (kDefByteArrayLength + sizeof(timeStamp))) {
        LOG(WARNING) << "Length of serialized buffer for NPW buffer value is too short: " << len;
        return false;
    }
    unsigned int i = 1; //first (0th) byte is buffer-type, we don't need that here

    std::memcpy(&(bufferId), serialBuff+i, sizeof(bufferId));
    i+= sizeof(bufferId);

    int temp = 0;

    for (unsigned int j = 0; j < (kDefNpwBufferLength); j++) {
        temp = (serialBuff[i + kHdrLen + 2 * j] << 8)
                + serialBuff[i + kHdrLen + 2 * j + 1];
        readingList[j] = temp;
    }
    i += kDefByteArrayLength;

    std::memcpy(&(timeStamp), serialBuff+i, sizeof(timeStamp));
    i+= sizeof(timeStamp);

    sensorId.clear();
    sensorId.append((char *)(&serialBuff[i]), len-i);
    return true;
}
