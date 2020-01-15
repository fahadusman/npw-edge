/*
 * StationEdgeDevice.h
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_EDGEDEVICE_H_
#define INCLUDE_EDGEDEVICE_H_

#include <list>
#include "PressureSensor.h"
#include "MqttHandler.h"

class EdgeDevice {
public:
    EdgeDevice();
    virtual ~EdgeDevice();
    void processIncomingCommand(CommandMsg * incomingCommand);
    void setHeartbeatInterval(int32_t hb);
    void runForever();
protected:
    std::list<Sensor *> sensorsList;  //List of PTs and TTs
    communicator * commPtr;
    uint32_t heartbeatInterval; //Heartbeat interval for edge device in MS
    bool keepRunning;
};

#endif /* INCLUDE_EDGEDEVICE_H_ */
