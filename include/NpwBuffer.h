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
const double KPTOffset = +1; // This is the initial offset applied to the value
                             // obtained from PT to compensate for negative values
                             // as they have to be transmitted as unsigned integers.
const double KPTScalingFactor = 1000; // for -1 to 100 PSI pressure range, and uint16
                             // it should be 648.

typedef double readingType;

class NpwBuffer: public CommDataBuffer {
private:
    readingType * readingList;
    unsigned int readingListLength;
    unsigned int byteArrayLength;
public:
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
