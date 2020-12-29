/*
 * PeriodicValue.h
 *
 *  Created on: Nov 27, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_PERIODICVALUE_H_
#define INCLUDE_PERIODICVALUE_H_

#include "CommDataBuffer.h"
#include "glog/logging.h"
#include <mutex>

class PeriodicValue: public CommDataBuffer {
private:
    double sensorValue;
    int sensorStatus;
public:
    PeriodicValue(double v, uint64_t t, const char * id, int s){
        sensorValue = v;
        timeStamp = t;
        strncpy(sensorId, id, sensorIdLen);
        sensorStatus = s;
    }
    std::string serializeJson() override;
    unsigned char * serialize(int & length) override;
    int deserialize(const unsigned char * serialBuff, const int & len);
    size_t getSerializedBuffLen() override;
};
#endif /* INCLUDE_PERIODICVALUE_H_ */
