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

NpwBuffer::NpwBuffer() {
    readingList = nullptr;
    readingListLength = 0;
    byteArrayLength = 0;
}

NpwBuffer::NpwBuffer(uint64_t ts, unsigned int readingListLen, std::string sId) {
    readingListLength = readingListLen;
    sensorId = sId;
    readingList = new (std::nothrow) readingType[readingListLen];
    LOG_IF(FATAL, readingList == nullptr) << "Unable to allocate memory for readingList";
    timeStamp = ts;
    byteArrayLength = 2*readingListLength + kHdrLen; //2 bytes value, plus the header

    LOG(INFO) << "new NPW Buffer, timeStamp: " << timeStamp << ", id: "
            << bufferId;
}

void NpwBuffer::insertAt(const unsigned int position, readingType value) {
    LOG_IF(FATAL,readingList == nullptr) << "readingList is not unallocated.";
    LOG_IF(FATAL,position > readingListLength) << "array index out of bound. index=" << position;

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
            new (std::nothrow) unsigned char[byteArrayLength]();
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

    for (unsigned int i = 0; i < readingListLength; i++) {
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
    writer.Key((sensorId + "_NPW_Array").c_str());
    writer.StartArray();
    for (unsigned int i = 0; i < byteArrayLength; i++)
        writer.Uint(byteArray[i]);
    writer.EndArray();
    writer.EndObject();

    delete byteArray;
    return s.GetString();
}

unsigned char * NpwBuffer::serialize(int & length) {
    unsigned char * serialBuffer = nullptr;
    try {
        length = 1/*buffer type*/+ sizeof(bufferId) + sizeof(byteArrayLength) + byteArrayLength
                + sizeof(timeStamp) + sizeof(expiryTime) + sensorId.length() + 1/*null char for sensorId*/;

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

        std::memcpy(serialBuffer+i, &(byteArrayLength), sizeof(byteArrayLength));
        i += sizeof(byteArrayLength);

        unsigned char * byteArray = createByteArray();
        std::memcpy(serialBuffer+i, byteArray, byteArrayLength);
        i += byteArrayLength;
        delete byteArray;

        std::memcpy(serialBuffer+i, &(timeStamp), sizeof(timeStamp));
        i+= sizeof(timeStamp);

        std::memcpy(serialBuffer+i, &(expiryTime), sizeof(expiryTime));
        i+= sizeof(expiryTime);

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
//    TODO: Determine length of byte array
    if ((unsigned int) len < (byteArrayLength + sizeof(timeStamp) + sizeof(expiryTime))) {
        LOG(WARNING) << "Length of serialized buffer for NPW buffer value is too short: " << len;
        return false;
    }
    unsigned int i = 1; //first (0th) byte is buffer-type, we don't need that here

    std::memcpy(&(bufferId), serialBuff+i, sizeof(bufferId));
    i+= sizeof(bufferId);

    std::memcpy(&(byteArrayLength), serialBuff+i, sizeof(byteArrayLength));
    i+= sizeof(byteArrayLength);

    readingListLength = (byteArrayLength-kHdrLen)/2;
    readingList = new (std::nothrow) readingType[readingListLength];
    LOG(INFO) << "readingListLength: " << readingListLength
            << "\tbyteArrayLength: " << byteArrayLength;

    if (readingList == nullptr) {
        LOG(ERROR) << "Unable to allocate memory for readingList";
        return false;
    }

    int temp = 0;
    for (unsigned int j = 0; j < (readingListLength); j++) {
        temp = (serialBuff[i + kHdrLen + 2 * j] << 8)
                + serialBuff[i + kHdrLen + 2 * j + 1];
        readingList[j] = temp;
    }
    i += byteArrayLength;

    std::memcpy(&(timeStamp), serialBuff+i, sizeof(timeStamp));
    i+= sizeof(timeStamp);

    std::memcpy(&(expiryTime), serialBuff+i, sizeof(expiryTime));
    i+= sizeof(expiryTime);

    sensorId.clear();
    sensorId.append((char *)(&serialBuff[i]), len-i);
    return true;
}

NpwBuffer::~NpwBuffer() {
    if (readingList != nullptr) {
        delete readingList;
    }
}
