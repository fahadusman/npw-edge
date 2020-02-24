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

/*
 *  An Edge device can have one of the three roles;
 *  * Block Valve site edge device that would communicate over
 *    serial radio link as a modbus slve
 *  * Station Edge device that would be installed at MW stations
 *    and send data to MQTT broker via MW link
 *  * Gateway edge device that would act as modbus master for
 *    station BV edevices to receive periodic and NPW data and
 *    then publish it on MQTT.
 */
enum Role {
    stationEdgeDevice,
    gatewayEdgeDevice,
    bvEdgeDevice
};

class EdgeDevice {
public:
    EdgeDevice(Role role);
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
    Role edgeDeviceRole;
};

#endif /* INCLUDE_EDGEDEVICE_H_ */
