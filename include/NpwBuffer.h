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
    NpwBuffer(uint64_t ts, unsigned int readingListLen, std::string sId);
    unsigned char * createByteArray();
    void insertAt(const unsigned int position, readingType value);

    std::string serializeJson() override;
    unsigned char * serialize(int & length) override;
    bool deserialize(const unsigned char *, const int & length) override;
    ~NpwBuffer();
};

#endif /* INCLUDE_NPWBUFFER_H_ */
