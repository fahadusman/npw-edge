/*
 * TemperatureSensor.h
 *
 *  Created on: Apr 1, 2020
 *      Author: Fahad Usman
 */

#ifndef TEMPERATURESENSOR_H_
#define TEMPERATURESENSOR_H_

#include "Sensor.h"
#include "SerialComPort.h"

const unsigned char kRKInitCommand[] = {1, 3, 0, 0, 0, 1, 132, 16};
const unsigned char kRKReadCommand[] = {1, 3, 0, 0, 0, 1, 132, 16};

class TemperatureSensor: public Sensor {
protected:
    SerialComPort sPort;
    bool recodringValues;
    std::thread * tempSensorThreadPtr;

public:
    TemperatureSensor(std::string portName, communicator *cPtr,
            EdgeDevice *ePtr, std::string sensorId);
    virtual ~TemperatureSensor();
    double readSensorValue();
    void initializeSensor();
    void temperatureSensorThread();
    void startThread();
    void stopThread();
};

#endif /* TEMPERATURESENSOR_H_ */
