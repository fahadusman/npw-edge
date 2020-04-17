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
        Sensor(cPtr, ePtr, temperatureSensorObj["sensor_id"].GetString()), sPort(
                temperatureSensorObj["port"].GetString(), B9600,
                kDefaultParity, kDefaultBlocking) {

    LOG_IF(FATAL, commPtr == nullptr) << "commPtr is null.";

    periodicValMinInterval = edgeDevicePtr->getRegisterValue(MIN_TIME_PERIODIC) * 1000;
    periodicValMaxInterval = edgeDevicePtr->getRegisterValue(MAX_TIME_PERIODIC) * 1000;
    periodicValChangeThreshold = edgeDevicePtr->getRegisterValue(ON_CHANG_THSH_TT);

    initializeSensor();
    recodringValues = false;
    tempSensorThreadPtr = nullptr;

    startThread();
}

TemperatureSensor::~TemperatureSensor() {

}

double TemperatureSensor::readSensorValue() {
    sPort.writeBuffer(kRKReadCommand, sizeof kRKReadCommand);
    unsigned char response[10];
    int bytesRead = sPort.readBuffer(response, sizeof response);

    if(bytesRead != 7){
        LOG(ERROR) << "Invalid number of bytes read from TT: " << bytesRead;
        return -0.1;
    }

    int16_t result = response[3] * 0xFF + response[4];

    return result/10;
}

void TemperatureSensor::initializeSensor(){
    LOG(INFO) << "Initialize TT, id: " << this->id;
    return;
}

void TemperatureSensor::temperatureSensorThread() {
    DLOG(INFO) << "starting TT thread\n";
    __uint64_t previousPeriodicValueTransmitTime = 0;
    double previousPeriodicVal = 0;
    double currentValue = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
            std::chrono::high_resolution_clock::now();

    std::chrono::milliseconds loopSleepInterval = std::chrono::milliseconds(1000);

    while(recodringValues){
        currentTimePoint = std::chrono::high_resolution_clock::now();
        currentValue = readSensorValue();
        currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTimePoint.time_since_epoch()).count();

        if (enablePeriodicValues) {
            previousPeriodicValueTransmitTime = sendPeriodicValue(currentTime,
                    previousPeriodicValueTransmitTime, previousPeriodicVal,
                    currentValue);
        }

        std::this_thread::sleep_for(loopSleepInterval);

    }
    DLOG(INFO) << "temperature sensor thread done.";
}

void TemperatureSensor::startThread(){
    DLOG(INFO) << "PressureSensor::startNpwThread()";
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
    LOG(INFO) << "PressureSensor::stopThread()";
    recodringValues = false;
    tempSensorThreadPtr->join();
    delete tempSensorThreadPtr;
    tempSensorThreadPtr = nullptr;
}
