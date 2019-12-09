#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

/*
 * sensor.h
 *
 *  Created on: Sep 25, 2019
 *      Author: Fahad Usman
 */
#include <string>
#include "communicator.h"

class Sensor {
public:
    virtual ~Sensor() = 0;
    virtual double readSensorValue() = 0;
    virtual void initializeSensor() = 0;
    Sensor(communicator * cptr);
protected:
    double currentValue;

    double periodicValChangeThreshold;
    unsigned int periodicValMinInterval;
    unsigned int periodicValMaxInterval;
    std::string id;
    communicator * commPtr;
};

inline Sensor::~Sensor() {
    // disconnect from sensor and close the socket
}

#endif /* INCLUDE_SENSOR_H_ */
