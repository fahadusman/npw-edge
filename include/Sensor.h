#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

/*
 * sensor.h
 *
 *  Created on: Sep 25, 2019
 *      Author: Fahad Usman
 */

#include <queue>
#include <modbus/modbus.h>
#include "rapidjson/document.h"

#include "communicator.h"
#include "CommandMsg.h"
#include "PeriodicValue.h"

class communicator;
class EdgeDevice;

enum SensorDataType {sdtInt16 = 0, sdtFloat};

class Sensor {
public:
    virtual ~Sensor();
    virtual double readSensorValue();
    virtual void initializeSensor();
    void disconnectSensor();
    Sensor(communicator * cptr, EdgeDevice * ePtr, const char * sensorId);
    virtual void enqueueCommand (CommandMsg *);
    CommandMsg * dequeueCommand();
    virtual PeriodicValue * getCurrentValue();
    bool enablePeriodicValues;
protected:
    modbus_t * sensorModbusCtx;
    std::string sensorPort;
    int64_t sensorBaudRate;
    double sensorScalingFactor;
    int sensorModbusSlaveId;
    int sensorModbusRegAddr;
    int sensorModbusNb;
    SensorDataType dataType;

    double currentValue;
    __uint64_t currentTime = 0;
    int currentStatus;
    double periodicValChangeThreshold;
    unsigned int periodicValMinInterval;
    unsigned int periodicValMaxInterval;
    char id[sensorIdLen];
    communicator * commPtr;
    std::queue <CommandMsg *> incomingCommandQueue;
    std::mutex commandQueueMutex;
    EdgeDevice * edgeDevicePtr;
    uint64_t sendPeriodicValue(uint64_t currentTime,
            uint64_t previousPeriodicValueTransmitTime,
            double & previousPeriodicVal, const double & currentValue);
    void parseSensorJsonObj(const rapidjson::Value & sensorObj);
};

#endif /* INCLUDE_SENSOR_H_ */
