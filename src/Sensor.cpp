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


Sensor::Sensor(communicator * cptr) {
    currentValue = 0;
    currentTime = 0;
    periodicValChangeThreshold = 0;
    periodicValMinInterval = 1000;
    periodicValMaxInterval = 5000;
    id = "defaultId"; //TODO: We need to come up with and identification hierarchy
    commPtr = cptr;
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
