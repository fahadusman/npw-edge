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

const unsigned int kHdrLen = 8;

typedef double readingType;

class NpwBuffer: public CommDataBuffer {
private:
    readingType * readingList;
    unsigned int readingListLength;
    unsigned int byteArrayLength;
public:
    static int32_t npwTimezoneOffset;
    uint64_t getTimestamp() {
        return timeStamp;
    }
    NpwBuffer();
    NpwBuffer(const uint64_t ts, const unsigned int readingListLen, const char * sId);
    unsigned char * createByteArray();
    void insertAt(const unsigned int position, readingType value);

    std::string serializeJson() override;
    unsigned char * serialize(int & length) override;
    int deserialize(const unsigned char *, const int & length) override;
    ~NpwBuffer();
    size_t getSerializedBuffLen() override;
};

#endif /* INCLUDE_NPWBUFFER_H_ */
