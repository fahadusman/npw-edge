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
    unsigned char * byteArray = new unsigned char[kDefByteArrayLength](); //2 bytes value, plus the header
//    std::time_t t = std::time(0);   // get time now
//    std::tm* npwTime = std::localtime(&t);

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
        int scaledVal = kScalingFactor * readingList[i];
        byteArray[2 * i + kHdrLen] = scaledVal >> 8;
        byteArray[2 * i + kHdrLen + 1] = scaledVal & 0xFF;
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

void * NpwBuffer::serialize(int & length) {
    length = kDefByteArrayLength;
    return createByteArray();
}
