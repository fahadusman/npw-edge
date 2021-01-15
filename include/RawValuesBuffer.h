/*
 * RawValuesBuffer.h
 *
 *  Created on: Jan 12, 2021
 *      Author: Fahad Usman
 */

#ifndef RAWVALUESBUFFER_H_
#define RAWVALUESBUFFER_H_

#include <string>
#include <vector>

class RawValuesBuffer {
public:
    RawValuesBuffer();
    RawValuesBuffer(const uint64_t &t, const std::string &sId,
            const unsigned int & sInterval, const std::vector<double> &l);
    virtual ~RawValuesBuffer();

    void storeInDatabase(uint64_t maxDuration);
    uint64_t timeStamp;
    std::string sensorId;
    unsigned int samplingIntervalMs;
    std::vector<double> valuesList;
};

#endif /* RAWVALUESBUFFER_H_ */
