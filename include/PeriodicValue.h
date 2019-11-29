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
public:
    PeriodicValue(double v, uint64_t t, std::string id){
        sensorValue = v;
        timeStamp = t;
        sensorId = id;
    }
    std::string serializeJson() override;
    void * serialize(int & length) override;
};
#endif /* INCLUDE_PERIODICVALUE_H_ */
