/*
 * TemperatureSensor.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: Fahad Usman
 */

#include "TemperatureSensor.h"

#include "EdgeDevice.h"

TemperatureSensor::TemperatureSensor(communicator *cPtr, EdgeDevice *ePtr,
        rapidjson::Value &temperatureSensorObj) :
        Sensor(cPtr, ePtr, temperatureSensorObj["sensor_id"].GetString()) {

    LOG_IF(FATAL, commPtr == nullptr) << "commPtr is null.";

    periodicValMinInterval = edgeDevicePtr->getRegisterValue(MIN_TIME_PERIODIC) * 1000;
    periodicValMaxInterval = edgeDevicePtr->getRegisterValue(MAX_TIME_PERIODIC) * 1000;
    periodicValChangeThreshold = edgeDevicePtr->getRegisterValue(ON_CHANG_THSH_TT);

    this->parseSensorJsonObj(temperatureSensorObj);
    recodringValues = false;
    tempSensorThreadPtr = nullptr;

    startThread();
}

TemperatureSensor::~TemperatureSensor() {

}

void TemperatureSensor::temperatureSensorThread() {
    LOG(INFO) << "starting TT thread\n";
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

        LOG_EVERY_N(INFO, 10) << "TemperatureSensor - id: " << id
                << ", v: " << currentValue << ", q: " << currentStatus
                << ", t: " << currentTime;

        std::this_thread::sleep_for(loopSleepInterval);
        processIncomingCommand();
    }
    LOG(INFO) << "temperature sensor thread done.";
}

void TemperatureSensor::startThread(){
    LOG(INFO) << "TemperatureSensor::startThread()";
    if (tempSensorThreadPtr != nullptr) {
        LOG(INFO) << "Stopping previous thread before starting new one.";
        stopThread();
        tempSensorThreadPtr = nullptr;
    }
    recodringValues = true;
    tempSensorThreadPtr = new std::thread(&TemperatureSensor::temperatureSensorThread, this);
    LOG(INFO) << "New Temperature Sensor thread launched.";
}

void TemperatureSensor::stopThread(){
    LOG(INFO) << "TemperatureSensor::stopThread()";
    recodringValues = false;
    tempSensorThreadPtr->join();
    delete tempSensorThreadPtr;
    tempSensorThreadPtr = nullptr;
}

void TemperatureSensor::processIncomingCommand() {
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
