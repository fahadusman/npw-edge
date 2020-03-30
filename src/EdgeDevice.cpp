/*
 * StationEdgeDevice.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: Fahad Usman
 */

#include <EdgeDevice.h>
#include <glog/logging.h>
#include <HeartbeatBuffer.h>
#include <chrono>

#include "DevConfig.h"
#include "CommDataBuffer.h"

EdgeDevice::EdgeDevice(int devId, Role role) :
        deviceId(devId), edgeDeviceRole(role) {
    LOG(INFO) << "EdgeDevice constructor";
    commPtr = nullptr;
    modbusMaster = nullptr;

    heartbeatInterval = kDcHeartbeatInterval.def;
    keepRunning = true;
    nextHBTimePoint = std::chrono::high_resolution_clock::now();
    initializeRegisterMap();
}

bool EdgeDevice::updateRegisterValue(CommandMsg *incomingCommand) {
    if (incomingCommand->getCommand() > UNINITIALIZED_CMD
            && incomingCommand->getCommand() < INVALID_COMMAND) {
        registerMap[incomingCommand->getCommand()] = incomingCommand->getData();
        return true;
    }
    LOG(FATAL) << "update register map, index out of bound";
    return false;
}

int32_t EdgeDevice::getRegisterValue(CommandRegister c) {
    LOG_IF(FATAL, c < 0 || c >= registerMapSize)
            << "Get register value, invalid out of bound";

    if (c >= 0 and c < registerMapSize) {
        return registerMap[c];
    }
    return 0;
}

void EdgeDevice::processIncomingCommand(CommandMsg * incomingCommand){
    if (edgeDeviceRole == gatewayEdgeDevice) {
        LOG(INFO) << "Gateway Edge device, going to enqueue command for modbus slave.";
        if (false == modbusMaster->enqueueSlaveCommand(incomingCommand))
        {
            LOG(WARNING) << "Failed to enqueue slave command.";
        }
        return;
    }

    updateRegisterValue(incomingCommand);

    switch (incomingCommand->getCommand()){
    case UNINITIALIZED_CMD:
        LOG(WARNING) << "Uninitialized command received.";
        delete incomingCommand;
        break;
    case REBOOT_TIME:
        //TODO: Implement reboot mechanism, for now we just exit the application
        keepRunning = false;
        delete incomingCommand;
        break;
    case HEARTBEAT_INTERVAL:
        setHeartbeatInterval(incomingCommand);
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
    case NPW_SAMPLE_BEFORE:
    case NPW_SAMPLE_AFTER:
        for (Sensor * sensorPtr : sensorsList) {
            sensorPtr->enqueueCommand(incomingCommand);
        }
        break;
    case ACK_NPW_BUFF:
        commPtr->removeMessageFromQueue(incomingCommand->getData());
        break;
    case INVALID_COMMAND:
    default:
        LOG(WARNING) << "Unhandled command received.";
        delete incomingCommand;
        incomingCommand = NULL;
    }
}

void EdgeDevice::setHeartbeatInterval(CommandMsg * cmd) {
    int32_t hb = cmd->getData();
    if (hb > kDcHeartbeatInterval.min and hb < kDcHeartbeatInterval.max) {
        heartbeatInterval = hb; //seconds
        updateRegisterValue(cmd);

    } else {
        LOG(WARNING) << "HEARTBEAT_INTERVAL value out of range";
    }
}

void EdgeDevice::runForever() {
    LOG(INFO) << "Entering edge device infinite loop.";
    while (keepRunning) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (edgeDeviceRole != bvEdgeDevice and
                nextHBTimePoint < std::chrono::high_resolution_clock::now()) {
            //Send heart beat
            DLOG(INFO) << "Sending Heartbeat.";
            CommDataBuffer * hb = getHeartBeat();
            commPtr->enqueueMessage(hb);
        }
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

void EdgeDevice::setModbusMaster(RadioCommunicator * modbusMasterPtr) {
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
    for (Sensor * sensorPtr : sensorsList) {
        delete sensorPtr;
        sensorPtr = NULL;
    }

    delete commPtr;
}

std::list<PeriodicValue*> EdgeDevice::getCurrentValues(){
    std::list<PeriodicValue*> valuesList;
    for (auto sensorPtr:sensorsList) {
        valuesList.push_back(sensorPtr->getCurrentValue());
    }
    return valuesList;
}

/*
 * Returns the current periodic value of each sensor of the Edge device.
 * Upon subsequent calls, it returns the value of next sensor in the sensorsList.
 */
PeriodicValue* EdgeDevice::getPeriodicSensorValue(){
    static auto sensorIt = sensorsList.begin();
    if (sensorIt == sensorsList.end()) {
        sensorIt = sensorsList.begin();
    }
    PeriodicValue * pValPtr = (*sensorIt++)->getCurrentValue();
    return pValPtr;
}

CommDataBuffer * EdgeDevice::getHeartBeat() {
    CommDataBuffer * hbPtr = nullptr;
    if(nextHBTimePoint < std::chrono::high_resolution_clock::now()) {
        hbPtr = new HeartbeatBuffer(deviceId, registerMap);
        nextHBTimePoint = std::chrono::high_resolution_clock::now()
                + std::chrono::seconds(heartbeatInterval);
        DLOG(INFO) << "Creating new heartbeat buffer";
    }
    return hbPtr;
}

void EdgeDevice::initializeRegisterMap() {
    for (unsigned int x = 0; x < registerMap.size(); x++) {
        registerMap[x] = 0;
    }
    registerMap[NPW_NUM_PACK] = kDcNpwNumPack.def;
    registerMap[NPW_EXP_TIME] = kDcNpwExpiryTime.def;
    registerMap[MAX_TIME_PERIODIC] = kDcMaxTimePeriodic.def;
    registerMap[MIN_TIME_PERIODIC] = kDcMinTimePeriodic.def;
    registerMap[ON_CHANG_THSH_PT] = kDcOnChangThshPt.def;
    registerMap[ON_CHANG_THSH_TT] = kDcOnChangThshTt.def;
    registerMap[SAMPLE_INTERVAL_NPW] = kDcSampleIntervalNpw.def;
    registerMap[NUM_SAMPLES_1_AVG] = kDcNumSamples1stAvg.def;
    registerMap[NUM_SAMPLES_2_AVG] = kDcNumSamples2ndAvg.def;
    registerMap[START_SAMPLE_2_AVG] = kDcStartSample2ndAvg.def;
    registerMap[NPW_THR_PT1] = kDcNpwPtThsh.def;
    registerMap[NPW_THR_PT2] = kDcNpwPtThsh.def;
    registerMap[NPW_THR_PT3] = kDcNpwPtThsh.def;
    registerMap[NPW_THR_PT4] = kDcNpwPtThsh.def;
    registerMap[NPW_SAMPLE_BEFORE] = kDcNpwSampleBefore.def;
    registerMap[NPW_SAMPLE_AFTER] = kDcNpwSampleAfter.def;
    registerMap[TEST_FLAG] = kDcTestFlag.def;
    registerMap[REBOOT_TIME] = kDcRebootTime.def;
    registerMap[HEARTBEAT_INTERVAL] = kDcHeartbeatInterval.def;
    //TODO: Check for saved values in file and apply those if exist
}
