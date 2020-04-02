/*
 * sensor.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: Fahad Usman
 */
#include "Sensor.h"

#include <string>
#include <mutex>
#include <exception>
#include <glog/logging.h>
#include <math.h>

#include "DevConfig.h"

Sensor::Sensor(communicator *cptr, EdgeDevice *eptr) :
        commPtr(cptr), edgeDevicePtr(eptr) {
    currentValue = 0;
    currentTime = 0;
    periodicValChangeThreshold = 0;
    periodicValMinInterval = kDcMinTimePeriodic.def;
    periodicValMaxInterval = kDcMaxTimePeriodic.def;
    id = "defaultId"; //TODO: We need to come up with and identification hierarchy
    enablePeriodicValues = false;
    return;
}

void Sensor::enqueueCommand(CommandMsg * cmd){
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        incomingCommandQueue.push(cmd);
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }
}

Sensor::~Sensor() {
    // disconnect from sensor and close the socket
    try{
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        while (not incomingCommandQueue.empty()){
            CommandMsg * c = incomingCommandQueue.front();
            LOG(INFO) << "Deleting command: " << c->getCommand();
            delete c;
            incomingCommandQueue.pop();
        }
    }
    catch (const std::exception & e) {
        LOG(ERROR) << "Exception: " << e.what();
    }
}

CommandMsg * Sensor::dequeueCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (incomingCommandQueue.empty()) {
            return nullptr;
        }
        CommandMsg * cptr = incomingCommandQueue.front();
        incomingCommandQueue.pop();
        return cptr;
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
        return nullptr;
    }
}

/*
 * Return the current periodic value as a pointer to dynamically
 * created object of PeriodicValue. It is the responsibility of the
 * caller to delete it after consuming the value.
 */
PeriodicValue * Sensor::getCurrentValue() {
    PeriodicValue * p = new PeriodicValue(currentValue, currentTime, id);
    return p;
}

uint64_t Sensor::sendPeriodicValue(uint64_t currentTime,
        uint64_t previousPeriodicValueTransmitTime,
        double & previousPeriodicVal, const double & currentValue) {
    if (((currentTime >= previousPeriodicValueTransmitTime + periodicValMinInterval)
            && (fabs(previousPeriodicVal - currentValue) >= periodicValChangeThreshold))
            || currentTime >= previousPeriodicValueTransmitTime + periodicValMaxInterval) {

        LOG_EVERY_N(INFO, 10) << "sending periodic value: " << currentValue;
        CommDataBuffer* pValBuffPtr = new PeriodicValue(currentValue,
                currentTime, id);
        commPtr->enqueueMessage(pValBuffPtr);
        previousPeriodicValueTransmitTime = currentTime;
        previousPeriodicVal = currentValue;
    }
    return previousPeriodicValueTransmitTime;
}
