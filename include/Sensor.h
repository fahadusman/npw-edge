#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

/*
 * sensor.h
 *
 *  Created on: Sep 25, 2019
 *      Author: Fahad Usman
 */

#include <queue>

#include "communicator.h"
#include "CommandMsg.h"
#include "PeriodicValue.h"

class communicator;

class Sensor {
public:
    virtual ~Sensor();
    virtual double readSensorValue() = 0;
    virtual void initializeSensor() = 0;
    Sensor(communicator * cptr);
    virtual void enqueueCommand (CommandMsg *);
    CommandMsg * dequeueCommand();
    PeriodicValue * getCurrentValue();
    bool enablePeriodicValues;
protected:
    double currentValue;
    __uint64_t currentTime = 0;

    double periodicValChangeThreshold;
    unsigned int periodicValMinInterval;
    unsigned int periodicValMaxInterval;
    std::string id;
    communicator * commPtr;
    std::queue <CommandMsg *> incomingCommandQueue;
    std::mutex commandQueueMutex;
};

#endif /* INCLUDE_SENSOR_H_ */
