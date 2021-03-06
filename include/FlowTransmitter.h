/*
 * FlowTransmitter.h
 *
 *  Created on: Feb 26, 2021
 *      Author: Fahad Usman
 *
 *  This class is specifically written for Krphne MFC400 Signal Converter
 *  for Mass Flow Transmitter
 */

#ifndef FLOWTRANSMITTER_H_
#define FLOWTRANSMITTER_H_

#include "Sensor.h"
#include "FtPeriodicValue.h"

class FlowTransmitter: public Sensor {
private:
    double flowVelociy, volumeFlow, massFlow, temperature, density;
    double totaliser1Value, totaliser2Value;
    uint16_t * eventGroups;
    uint16_t measurementValueAddress, totalizerAddress,
            eventGroupsAddress;
    uint16_t measurementValueNb, totalizerNb,
            eventGroupsNb;

    bool recodringValues;
    std::thread * flowtransmitterThreadPtr;

    uint64_t sendPeriodicValue(uint64_t currentTime,
            uint64_t previousPeriodicValueTransmitTime,
            double &previousPeriodicVal, const double &currentValue);
    PeriodicValue * getCurrentValue();
    void processIncomingCommand();
public:
    FlowTransmitter(communicator *cPtr, EdgeDevice *ePtr,
            rapidjson::Value & ftObj);
    virtual ~FlowTransmitter();
    double readSensorValue();
    void flowtTransmitterThread();
    void startThread();
    void stopThread();
};

#endif /* FLOWTRANSMITTER_H_ */
