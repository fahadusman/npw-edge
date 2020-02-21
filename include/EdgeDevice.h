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
#include "RadioCommunicator.h"

class EdgeDevice {
public:
    EdgeDevice();
    virtual ~EdgeDevice();
    void processIncomingCommand(CommandMsg * incomingCommand);
    void setHeartbeatInterval(int32_t hb);
    void runForever();
    void addSensor(Sensor * sensorPtr);
    void setCommunicator(communicator * cPtr);
    void setModbusMaster(communicator * modbusMasterPtr);
    int sendMessage(CommDataBuffer * d);
protected:
    std::list<Sensor *> sensorsList;  //List of PTs and TTs
    communicator * commPtr;
    uint32_t heartbeatInterval; //Heartbeat interval for edge device in MS
    bool keepRunning;
    communicator * modbusMaster;
};

#endif /* INCLUDE_EDGEDEVICE_H_ */
