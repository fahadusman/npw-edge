/*
 * StationEdgeDevice.h
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_STATIONEDGEDEVICE_H_
#define INCLUDE_STATIONEDGEDEVICE_H_

#include <list>
#include "PressureSensor.h"
#include "MqttHandler.h"

class StationEdgeDevice {
public:
    StationEdgeDevice();
    virtual ~StationEdgeDevice();
protected:
    std::list<Sensor *> sensorsList;  //List of PTs and TTs
    communicator * commPtr;
};

#endif /* INCLUDE_STATIONEDGEDEVICE_H_ */
