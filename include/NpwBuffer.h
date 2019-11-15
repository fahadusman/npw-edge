/*
 * NpwBuffer.h
 *
 *  Created on: Oct 18, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_NPWBUFFER_H_
#define INCLUDE_NPWBUFFER_H_

#include <glog/logging.h>

const unsigned int kDefNpwBufferLength = 500; //TODO: it should be samples per second (50) x npw buffer duration (10s) = 500,
const unsigned int kDefT1Ms = 3000;
const unsigned int kDefT2Ms = 7000;

const double kScalingFactor = 0.01;
const unsigned int kHdrLen = 8;
const unsigned int kDefByteArrayLength = kDefNpwBufferLength*2 + kHdrLen;

typedef double readingType;

class NpwBuffer {
private:
    uint64_t timeStamp;
    readingType readingList[kDefNpwBufferLength];
public:
    uint64_t getTimestamp() {
        return timeStamp;
    }
    NpwBuffer() :
            readingList { } {
        timeStamp = 0;
    }
    NpwBuffer(uint64_t ts) :
            readingList { } {
        timeStamp = ts;
        LOG(INFO) << "new NPW Buffer, timeStamp: " << timeStamp;
    }
    readingType & operator [](const unsigned int index) {
        if (index >= kDefNpwBufferLength) {
            LOG(FATAL) << "array index out of bound. index=" << index;
            return readingList[0];
        }
        return readingList[index];
    }

    void insertAt(const unsigned int position, readingType value) {
        if (position >= kDefNpwBufferLength) {
            LOG(FATAL) << "array index out of bound. index=" << position;
        }
        readingList[position] = value;
    }

    unsigned char * createByteArray();

    std::string serialize();
};

#endif /* INCLUDE_NPWBUFFER_H_ */
