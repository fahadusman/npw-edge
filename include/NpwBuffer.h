/*
 * NpwBuffer.h
 *
 *  Created on: Oct 18, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_NPWBUFFER_H_
#define INCLUDE_NPWBUFFER_H_

#include <glog/logging.h>

#include "CommDataBuffer.h"

const unsigned int kDefNpwBufferLength = 500; //TODO: it should be samples per second (50) x npw buffer duration (10s) = 500,
const unsigned int kDefT1Ms = 3000;
const unsigned int kDefT2Ms = 7000;

const double kScalingFactor = 1000;
const unsigned int kHdrLen = 8;

typedef double readingType;

class NpwBuffer: public CommDataBuffer {
private:
    readingType readingList[kDefNpwBufferLength];
public:
    uint64_t getTimestamp() {
        return timeStamp;
    }

    NpwBuffer();
    NpwBuffer(uint64_t ts);
    unsigned char * createByteArray();
    void insertAt(const unsigned int position, readingType value);

    std::string serializeJson() override;
    void * serialize(int & length) override;
};

#endif /* INCLUDE_NPWBUFFER_H_ */
