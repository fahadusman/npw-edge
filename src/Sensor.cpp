/*
 * sensor.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: Fahad Usman
 */
#include "Sensor.h"

Sensor::Sensor(communicator * cptr) {
    currentValue = 0;
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

CommandMsg * Sensor::dequeueCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (incomingCommandQueue.empty()) {
            return NULL;
        }
        CommandMsg * cptr = incomingCommandQueue.front();
        incomingCommandQueue.pop();
        return cptr;
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
        return NULL;
    }
}
