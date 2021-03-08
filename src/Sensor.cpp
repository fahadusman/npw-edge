/*
 * sensor.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: Fahad Usman
 */
#include "Sensor.h"

#include <string>
#include <mutex>
#include <exception>
#include <glog/logging.h>
#include <math.h>
#include <string.h>

#include "DevConfig.h"

Sensor::Sensor(communicator *cptr, EdgeDevice *eptr, const char * sensorId) :
        commPtr(cptr), edgeDevicePtr(eptr) {
    currentValue = 0;
    currentTime = 0;
    periodicValChangeThreshold = 0;
    periodicValMinInterval = kDcMinTimePeriodic.def;
    periodicValMaxInterval = kDcMaxTimePeriodic.def;
    if (sensorId == nullptr or strlen(sensorId) == 0) {
        strncpy(id, "defaultId", sensorIdLen);
        LOG(FATAL) << "Invalid SensorId";
    } else {
        strncpy(id, sensorId, sensorIdLen);
    }
    enablePeriodicValues = false;
    currentStatus = 0;
    sensorModbusCtx = nullptr;
    sensorPort = "";
    sensorBaudRate = 115200;
    sensorScalingFactor = 1;
    sensorModbusSlaveId = 1;
    sensorModbusRegAddr = 2;
    sensorModbusNb = 2;
    dataType = sdtFloat;
    return;
}
void Sensor::enqueueCommand(CommandMsg * cmd){
    try {
        CommandMsg * tempcmd = new CommandMsg;
        *tempcmd = * cmd;
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        incomingCommandQueue.push(tempcmd);
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }
}

Sensor::~Sensor() {
    if (sensorModbusCtx != nullptr) {
        disconnectSensor();
    }

    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        while (not incomingCommandQueue.empty()) {
            CommandMsg *c = incomingCommandQueue.front();
            LOG(INFO) << "Deleting command: " << c->getCommand();
            delete c;
            incomingCommandQueue.pop();
        }
    } catch (const std::exception &e) {
        LOG(ERROR) << "Exception: " << e.what();
    }
}

CommandMsg * Sensor::dequeueCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (incomingCommandQueue.empty()) {
            return nullptr;
        }
        CommandMsg * cptr = incomingCommandQueue.front();
        incomingCommandQueue.pop();
        return cptr;
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
        return nullptr;
    }
}

/*
 * Return the current periodic value as a pointer to dynamically
 * created object of PeriodicValue. It is the responsibility of the
 * caller to delete it after consuming the value.
 */
PeriodicValue * Sensor::getCurrentValue() {
    PeriodicValue * p = new PeriodicValue(currentValue, currentTime, id, currentStatus);
    return p;
}

uint64_t Sensor::sendPeriodicValue(uint64_t currentTime,
        uint64_t previousPeriodicValueTransmitTime,
        double & previousPeriodicVal, const double & currentValue) {
    if (((currentTime >= previousPeriodicValueTransmitTime + periodicValMinInterval)
            && (fabs(previousPeriodicVal - currentValue) >= periodicValChangeThreshold))
            || currentTime >= previousPeriodicValueTransmitTime + periodicValMaxInterval) {

        LOG_EVERY_N(INFO, 10) << "sending periodic value: " << currentValue;
        CommDataBuffer* pValBuffPtr = getCurrentValue();
        commPtr->enqueueMessage(pValBuffPtr);
        previousPeriodicValueTransmitTime = currentTime;
        previousPeriodicVal = currentValue;
    }
    return previousPeriodicValueTransmitTime;
}

void Sensor::parseSensorJsonObj(const rapidjson::Value & sensorObj) {
    try {
        sensorPort = sensorObj["port"].GetString();
        strncpy(id, sensorObj["sensor_id"].GetString(), sensorIdLen);
        id[sensorIdLen-1] = '\0';
        sensorBaudRate = sensorObj["baud_rate"].GetInt64();
        sensorScalingFactor = sensorObj["sensor_scaling_factor"].GetFloat();
        std::string dataTypeStr = sensorObj["sensor_data_type"].GetString();
        if (dataTypeStr == "int16") {
            dataType = sdtInt16;
        } else if (dataTypeStr == "float") {
            dataType = sdtFloat;
        } else {
            dataType = sdtMultiple;
        }
        sensorModbusSlaveId = sensorObj["modbus_slave_id"].GetInt();
        sensorModbusRegAddr = sensorObj["modbus_reg_addr"].GetInt();
        sensorModbusNb = sensorObj["modbus_nb"].GetInt();

    } catch (std::exception & e) {
        LOG(FATAL) << "exception: " << e.what();
    }

}

void Sensor::initializeSensor() {
    while (sensorModbusCtx == nullptr) {
        sensorModbusCtx = modbus_new_rtu(sensorPort.c_str(), sensorBaudRate, 'N', 8, 1);
        if (sensorModbusCtx == nullptr) {
            LOG(ERROR) << "Unable to create the libmodbus context, port: "
                    << sensorPort;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }
        modbus_set_slave(sensorModbusCtx, sensorModbusSlaveId);
        if (modbus_connect(sensorModbusCtx) == -1) {
            LOG(ERROR) << "Modbus Connection failed, id: " << id
                    << ", port: " << sensorPort
                    << ", Error: " << modbus_strerror(errno);

            modbus_free(sensorModbusCtx);
            sensorModbusCtx = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }
    }
    LOG(INFO) << "Initialized sensor: " << id;
}

double Sensor::readSensorValue(){
    int rc;
    uint16_t tabReg[16];
    LOG_IF(FATAL, sensorModbusNb > 16) << "Number registers to read is too large";

    if (sensorModbusCtx == nullptr) {
        initializeSensor();
    }
    rc = modbus_read_registers(sensorModbusCtx, sensorModbusRegAddr, sensorModbusNb, tabReg);
    if (rc == -1) {
        LOG_EVERY_N(ERROR, 2) << "Failed to read from PT, id:(" << id
                << ") Error: " << modbus_strerror(errno);
        currentStatus = 0;
        disconnectSensor();
        return currentValue; //return previous current value
    }

    if (dataType == sdtInt16) {
        currentValue = int16_t(tabReg[0]) * sensorScalingFactor;
    } else {
        float result = 0.0;
        unsigned char * resultPtr= reinterpret_cast<unsigned char*>(&result);
        unsigned char * responsePtr= reinterpret_cast<unsigned char*>(tabReg);


        resultPtr[0] = responsePtr[2];
        resultPtr[1] = responsePtr[3];
        resultPtr[2] = responsePtr[0];
        resultPtr[3] = responsePtr[1];

        currentValue = result * sensorScalingFactor;
    }

    currentStatus = 1;
    return currentValue;
}

void Sensor::disconnectSensor() {
    LOG(INFO) << "Disconnecting modbus sensor, " << id;
    modbus_close(sensorModbusCtx);
    modbus_free(sensorModbusCtx);
    sensorModbusCtx = nullptr;
}

float extractFloat (unsigned char * startAddr) {
    float result = 0.0;
    unsigned char * resultPtr= reinterpret_cast<unsigned char*>(&result);

    resultPtr[0] = startAddr[2];
    resultPtr[1] = startAddr[3];
    resultPtr[2] = startAddr[0];
    resultPtr[3] = startAddr[1];

    return result;
}
