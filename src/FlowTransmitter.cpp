/*
 * FlowTransmitter.cpp
 *
 *  Created on: Feb 26, 2021
 *      Author: Fahad Usman
 */

#include <glog/logging.h>
#include <math.h>

#include "FlowTransmitter.h"
#include "EdgeDevice.h"

FlowTransmitter::FlowTransmitter(communicator *cPtr, EdgeDevice *ePtr,
        rapidjson::Value &flowTransmitterObj) :
        Sensor(cPtr, ePtr, flowTransmitterObj["sensor_id"].GetString()) {
    // TODO Auto-generated constructor stub
    flowVelociy = volumeFlow =  massFlow = temperature = density = 0.0;
    totaliser1Value = totaliser2Value = 0;

    periodicValMinInterval = edgeDevicePtr->getRegisterValue(MIN_TIME_PERIODIC) * 1000;
    periodicValMaxInterval = edgeDevicePtr->getRegisterValue(MAX_TIME_PERIODIC) * 1000;
    periodicValChangeThreshold = edgeDevicePtr->getRegisterValue(ON_CHANG_THSH_TT);

    parseSensorJsonObj(flowTransmitterObj);
    flowtransmitterThreadPtr = nullptr;
    recodringValues = false;

    measurementValueAddress = sensorModbusRegAddr;
    measurementValueNb = sensorModbusNb;

    //TODO: Get totalizer and event groups configurations for JSON file
    totalizerAddress = 32100;
    totalizerNb = 4;

    eventGroupsAddress = 52016;
    eventGroupsNb = 8;
    eventGroups = new uint16_t[eventGroupsNb];

    parity = 'E';

    startThread();
}

FlowTransmitter::~FlowTransmitter() {
    delete eventGroups;
    stopThread();
}

double FlowTransmitter::readSensorValue() {
    currentValue = currentValue +1;
    volumeFlow = currentValue;
    currentStatus = 1;
    return currentValue;

    int rc;
    uint16_t tabReg[16];
    LOG_IF(FATAL, sensorModbusNb > 16) << "Number registers to read is too large";

    if (sensorModbusCtx == nullptr) {
        initializeSensor();
    }
    rc = modbus_read_input_registers(sensorModbusCtx, measurementValueAddress,
            measurementValueNb, tabReg);
    if (rc == -1) {
        LOG_EVERY_N(ERROR, 2) << "Failed to read from PT, id:(" << id
                << ") Error: " << modbus_strerror(errno);
        currentStatus = 0;
        disconnectSensor();
        return currentValue; //return previous current value
    }

    flowVelociy = extractFloat(reinterpret_cast<unsigned char*>(tabReg));
    currentValue = volumeFlow =  extractFloat(reinterpret_cast<unsigned char*>(tabReg+4));
    massFlow = extractFloat(reinterpret_cast<unsigned char*>(tabReg+8));
    temperature = extractFloat(reinterpret_cast<unsigned char*>(tabReg+12));
    density = extractFloat(reinterpret_cast<unsigned char*>(tabReg+16));

    rc = modbus_read_input_registers(sensorModbusCtx, totalizerAddress,
            totalizerNb, tabReg);
    if (rc == -1) {
        LOG_EVERY_N(ERROR, 2) << "Failed to read from PT, id:(" << id
                << ") Error: " << modbus_strerror(errno);
        currentStatus = 0;
        disconnectSensor();
        return currentValue; //return previous current value
    }

    totaliser1Value = extractFloat(reinterpret_cast<unsigned char*>(tabReg));
    totaliser2Value = extractFloat(reinterpret_cast<unsigned char*>(tabReg));

    rc = modbus_read_registers(sensorModbusCtx, eventGroupsAddress,
            eventGroupsNb, tabReg);
    if (rc == -1) {
        LOG_EVERY_N(ERROR, 2) << "Failed to read from PT, id:(" << id
                << ") Error: " << modbus_strerror(errno);
        currentStatus = 0;
        disconnectSensor();
        return currentValue; //return previous current value
    }

    return currentValue;
}

void FlowTransmitter::flowtTransmitterThread() {
    LOG(INFO) << "starting Flow Transmitter thread\n";
    __uint64_t previousPeriodicValueTransmitTime = 0;
    double previousPeriodicVal = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
            std::chrono::high_resolution_clock::now();

    std::chrono::milliseconds loopSleepInterval = std::chrono::milliseconds(1000);

    while(recodringValues){
        currentTimePoint = std::chrono::high_resolution_clock::now();
        currentValue = readSensorValue();
        currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTimePoint.time_since_epoch()).count();

        if (currentStatus == 0) {
            currentValue = previousPeriodicVal;
        }

        if (enablePeriodicValues) {
            previousPeriodicValueTransmitTime = sendPeriodicValue(currentTime,
                    previousPeriodicValueTransmitTime, previousPeriodicVal,
                    currentValue);
        }

        LOG_EVERY_N(INFO, 10) << "FlowTransmitter - id: " << id
                << ", v: " << currentValue << ", q: " << currentStatus
                << ", t: " << currentTime;

        std::this_thread::sleep_until(currentTimePoint + loopSleepInterval);
        processIncomingCommand();
    }
    LOG(INFO) << "Flow Transmitter thread done.";

}

void FlowTransmitter::startThread(){
    LOG(INFO) << "FlowTransmitter::startThread()";
    if (flowtransmitterThreadPtr != nullptr) {
        LOG(INFO) << "Stopping previous thread before starting new one.";
        stopThread();
        flowtransmitterThreadPtr = nullptr;
    }
    recodringValues = true;
    flowtransmitterThreadPtr = new std::thread(&FlowTransmitter::flowtTransmitterThread, this);
    LOG(INFO) << "New FlowTransmitter Sensor thread launched.";
}

void FlowTransmitter::stopThread(){
    LOG(INFO) << "FlowTransmitter::stopThread()";
    recodringValues = false;
    flowtransmitterThreadPtr->join();
    delete flowtransmitterThreadPtr;
    flowtransmitterThreadPtr = nullptr;
}

//TODO: refactor sendPeriodicValue function to use from the base class
uint64_t FlowTransmitter::sendPeriodicValue(uint64_t currentTime,
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

/*
 * Return the current periodic value as a pointer to dynamically
 * created object of PeriodicValue. It is the responsibility of the
 * caller to delete it after consuming the value.
 */
PeriodicValue * FlowTransmitter::getCurrentValue() {
    FtPeriodicValue *p = new FtPeriodicValue(flowVelociy, volumeFlow, massFlow,
            temperature, density, totaliser1Value, totaliser2Value, eventGroups,
            eventGroupsNb, currentTime, id, currentStatus);
    return p;
}

// TODO: handling of MIN_TIME_PERIODIC and MAX_TIME_PERIODIC commands should be
// part of Sensor base class
void FlowTransmitter::processIncomingCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (not incomingCommandQueue.empty()) {
            CommandMsg *c = incomingCommandQueue.front();
            LOG(INFO) << "Processing command: " << c->getCommand()
                    << "\tData: " << c->getData();

            switch (c->getCommand()) {
            case MAX_TIME_PERIODIC:
                if (c->getData() >= kDcMaxTimePeriodic.min
                        and c->getData() <= kDcMaxTimePeriodic.max) {
                    periodicValMaxInterval = c->getData() * 1000;
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "MAX_TIME_PERIODIC value out of range: "
                            << c->getData();
                break;
            case MIN_TIME_PERIODIC:
                if (c->getData() >= kDcMinTimePeriodic.min
                        and c->getData() <= kDcMinTimePeriodic.max) {
                    this->periodicValMinInterval = c->getData() * 1000;
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "MIN_TIME_PERIODIC value out of range: "
                            << c->getData();
                break;
            case ON_CHANG_THSH_TT:
                if (c->getData() >= kDcOnChangThshTt.min
                        and c->getData() <= kDcOnChangThshTt.max) {
                    this->periodicValChangeThreshold = c->getData();
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "ON_CHANG_THSH_TT value out of range: "
                            << c->getData();
                break;
            default:
                LOG(INFO) << "Command not applicable for TT. cmd: "
                        << c->getCommand();
            }
            incomingCommandQueue.pop();
            delete c;
        }
    } catch (const std::exception &e) {
        LOG(ERROR) << "Exception: " << e.what();
    }
}
