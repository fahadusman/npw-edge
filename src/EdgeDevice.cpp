/*
 * StationEdgeDevice.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#include <EdgeDevice.h>
#include <glog/logging.h>

#include "DevConfig.h"
#include "CommDataBuffer.h"

EdgeDevice::EdgeDevice(Role role) {
    edgeDeviceRole = role;
    LOG(INFO) << "EdgeDevice constructor";
    commPtr = nullptr;
    modbusMaster = nullptr;

    heartbeatInterval = kDcHeartbeatInterval.def;
    keepRunning = true;
}

void EdgeDevice::processIncomingCommand(CommandMsg * incomingCommand){
    switch (incomingCommand->getCommand()){
    case UNINITIALIZED_CMD:
        LOG(WARNING) << "Uninitialized command received.";
        delete incomingCommand;
        break;
    case REBOOT_TIME:
        //TODO: Implement reboot mechanism
        keepRunning = false;
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

void EdgeDevice::setHeartbeatInterval(int32_t hb) {
    if (hb > kDcHeartbeatInterval.min and hb < kDcHeartbeatInterval.max) {
        heartbeatInterval = hb * 1000; //convert it from seconds to milliseconds
    } else {
        LOG(WARNING) << "HEARTBEAT_INTERVAL value out of range";
    }
}

void EdgeDevice::runForever() {
    LOG(INFO) << "Entering edge device infinite loop.";
    while (keepRunning) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    LOG(WARNING) << "keepRunning loop terminated";
}

void EdgeDevice::addSensor(Sensor * sensorPtr) {
    sensorsList.push_back(sensorPtr);
}
void EdgeDevice::setCommunicator(communicator * cPtr) {
    if (commPtr != nullptr) {
        delete commPtr;
        commPtr = nullptr;
    }
    commPtr = cPtr;
}

void EdgeDevice::setModbusMaster(communicator * modbusMasterPtr) {
    if (modbusMaster != nullptr) {
        delete modbusMaster;
        modbusMaster = nullptr;
    }
    modbusMaster = modbusMasterPtr;
}

int EdgeDevice::sendMessage(CommDataBuffer * d) {
    if (commPtr == nullptr) {
        LOG(FATAL) << "commPtr not set for Edge Device. Cannot send message.";
        return 0;
    }
    return commPtr->enqueueMessage(d);
}

EdgeDevice::~EdgeDevice() {
    // TODO Auto-generated destructor stub
    for (Sensor * sensorPtr : sensorsList) {
        delete sensorPtr;
        sensorPtr = NULL;
    }

    delete commPtr;
}

