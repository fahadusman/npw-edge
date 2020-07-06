/*
 * StationEdgeDevice.h
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_EDGEDEVICE_H_
#define INCLUDE_EDGEDEVICE_H_

#include <list>
#include <array>

#include "PressureSensor.h"
#include "MqttHandler.h"
#include "RadioCommunicator.h"
#include "HeartbeatBuffer.h"

class RadioCommunicator;
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
    EdgeDevice(const char * confFilePath);
    virtual ~EdgeDevice();
    void processIncomingCommand(CommandMsg * incomingCommand);
    void processIncomingCommand(std::string registerName, uint32_t data);
    void setHeartbeatInterval(CommandMsg * cmd);
    void runForever();
    void addSensor(Sensor * sensorPtr);
    void setCommunicator(communicator * cPtr);
    void setModbusMaster(RadioCommunicator * modbusMasterPtr);
    int sendMessage(CommDataBuffer * d);
    std::list<PeriodicValue*> getCurrentValues();
    PeriodicValue* getPeriodicSensorValue();
    Role getRole() {
        return edgeDeviceRole;
    }
    bool sendSlaveCommand();
    CommDataBuffer * getHeartBeat();
    bool updateRegisterValue(CommandMsg *incomingCommand);
    int32_t getRegisterValue(CommandRegister c);
    bool addConfigToConfigMqp(const std::string registerName,
            uint8_t deviceId, CommandRegister cmdReg);
    int getDeviceId() { return deviceId; }
protected:
    std::string deviceName;
    int deviceId;
    std::list<Sensor *> sensorsList;  //List of PTs and TTs
    communicator * commPtr;
    uint32_t heartbeatInterval; //Heartbeat interval for edge device in seconds
    bool keepRunning;
    RadioCommunicator * modbusMaster;
    Role edgeDeviceRole;
    std::chrono::time_point<std::chrono::high_resolution_clock> nextHBTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> applicationStartTime;
    std::array <int32_t, registerMapSize> registerMap;
    void initializeRegisterMap();
    bool loadRegisterMapFromFile();
    bool saveRegisterMapToFile();
    char * readConfigFile(const char *confFilePath);
    void initializeConfigMap();

    std::map <std::string, CommandMsg> configMap;

};

#endif /* INCLUDE_EDGEDEVICE_H_ */
