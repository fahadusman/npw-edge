/*
 * StationEdgeDevice.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#include <StationEdgeDevice.h>

#include <glog/logging.h>

#include "DevConfig.h"

StationEdgeDevice::StationEdgeDevice() {
    LOG(INFO) << "StationEdgeDevice constructor";
    commPtr = new MqttCommunicator(this);
    commPtr->connect();

    PressureSensor * sensorPtr = new PressureSensor("/dev/ttyUSB0", commPtr);
    sensorPtr->startNpwThread();
    sensorsList.push_back(sensorPtr);
    heartbeatInterval = kDcHeartbeatInterval.def;
}

void StationEdgeDevice::processIncomingCommand(CommandMsg * incomingCommand){
    switch (incomingCommand->getCommand()){
    case UNINITIALIZED_CMD:
        LOG(WARNING) << "Uninitialized command received.";
        delete incomingCommand;
        break;
    case REBOOT_TIME:
        //TODO: Implement reboot mechanism
        delete incomingCommand;
        break;
    case HEARTBEAT_INTERVAL:
        setHeartbeatInterval(incomingCommand->getData());
        delete incomingCommand;
        break;
    case MAX_TIME_PERIODIC:
    case MIN_TIME_PERIODIC:
    case ON_CHANG_THSH_PT:
    case NPW_EXP_TIME:
    case TEST_FLAG:

    case SAMPLE_INTERVAL_NPW:
    case NUM_SAMPLES_1_AVG:
    case NUM_SAMPLES_2_AVG:
    case START_SAMPLE_2_AVG:
    case NPW_THR_PT1:
    case NPW_THR_PT2:
    case NPW_THR_PT3:
    case NPW_THR_PT4:
        for (Sensor * sensorPtr : sensorsList) {
            sensorPtr->enqueueCommand(incomingCommand);
        }
        break;
    case INVALID_COMMAND:
    default:
        LOG(WARNING) << "Unhandled command received.";
        delete incomingCommand;
        incomingCommand = NULL;
    }
}

void StationEdgeDevice::setHeartbeatInterval(int32_t hb) {
    if (hb > kDcHeartbeatInterval.min and hb < kDcHeartbeatInterval.max) {
        heartbeatInterval = hb * 1000; //convert it from seconds to milliseconds
    } else {
        LOG(WARNING) << "HEARTBEAT_INTERVAL value out of range";
    }
}
StationEdgeDevice::~StationEdgeDevice() {
    // TODO Auto-generated destructor stub
    for (Sensor * sensorPtr : sensorsList) {
        delete sensorPtr;
        sensorPtr = NULL;
    }

    delete commPtr;
}

