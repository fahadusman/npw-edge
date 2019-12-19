/*
 * StationEdgeDevice.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#include <StationEdgeDevice.h>

StationEdgeDevice::StationEdgeDevice() {
    LOG(INFO) << "StationEdgeDevice constructor";
    commPtr = new MqttCommunicator(this);
    commPtr->connect();

    PressureSensor * sensorPtr = new PressureSensor("/dev/ttyUSB0", commPtr);
    sensorPtr->startNpwThread();
    sensorsList.push_back(sensorPtr);

}

StationEdgeDevice::~StationEdgeDevice() {
    // TODO Auto-generated destructor stub
    for (Sensor * sensorPtr : sensorsList) {
        delete sensorPtr;
        sensorPtr = NULL;
    }

    delete commPtr;
}

