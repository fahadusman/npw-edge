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
    void processIncomingCommand(CommandMsg * incomingCommand);
    void setHeartbeatInterval(int32_t hb);
protected:
    std::list<Sensor *> sensorsList;  //List of PTs and TTs
    communicator * commPtr;
    uint32_t heartbeatInterval; //Heartbeat interval for edge device in MS
};

#endif /* INCLUDE_STATIONEDGEDEVICE_H_ */
