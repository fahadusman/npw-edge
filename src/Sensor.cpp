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
