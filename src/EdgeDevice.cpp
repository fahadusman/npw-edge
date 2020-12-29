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
#include <fstream>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"


#include "DevConfig.h"
#include "CommDataBuffer.h"
#include "TemperatureSensor.h"
#include "NpwBuffer.h"

const char * kRegMapFileName = "reg_map.bin";
const char * kConfigFileName = "config.json";

char * EdgeDevice::readConfigFile(const char *confFilePath) {
    char *configFileReadBuffer = nullptr;
    int fileLength;
    std::ifstream configFile(confFilePath);
    if (configFile.is_open()) {
        configFile.seekg(0, configFile.end);
        fileLength = configFile.tellg();
        configFileReadBuffer = new (std::nothrow) char[fileLength+1]; //+1 for terminating null char
        LOG_IF(FATAL, configFileReadBuffer == nullptr) << "Failed to allocate "
                << "buffer to read config file, size: " << fileLength;
        configFile.seekg(0, configFile.beg);
        configFile.read(configFileReadBuffer, fileLength);
        if (configFile.good()) {
            configFileReadBuffer[fileLength] = '\0';
            LOG(INFO) << "Successfully read device config file.";
        } else {
            LOG(FATAL)
                    << "Unable to read complete register map, bytes count: "
                    << configFile.gcount();
        }
    } else {
        LOG(FATAL) << "Unable to open device configuration file.";
    }
    configFile.close();
    return configFileReadBuffer;
}

void EdgeDevice::initializeConfigMap() {
    configMap["UNINITIALIZED_CMD"] = UNINITIALIZED_CMD;
    configMap["NPW_NUM_PACK"] = NPW_NUM_PACK;
    configMap["NPW_EXP_TIME"] = NPW_EXP_TIME;
    configMap["MAX_TIME_PERIODIC"] = MAX_TIME_PERIODIC;
    configMap["MIN_TIME_PERIODIC"] = MIN_TIME_PERIODIC;
    configMap["ON_CHANG_THSH_PT"] = ON_CHANG_THSH_PT;
    configMap["ON_CHANG_THSH_TT"] = ON_CHANG_THSH_TT;
    configMap["SAMPLE_INTERVAL_NPW"] = SAMPLE_INTERVAL_NPW;
    configMap["NUM_SAMPLES_1_AVG"] = NUM_SAMPLES_1_AVG;
    configMap["NUM_SAMPLES_2_AVG"] = NUM_SAMPLES_2_AVG;
    configMap["START_SAMPLE_2_AVG"] = START_SAMPLE_2_AVG;
    configMap["NPW_SAMPLE_BEFORE"] = NPW_SAMPLE_BEFORE;
    configMap["NPW_SAMPLE_AFTER"] = NPW_SAMPLE_AFTER;
    configMap["TEST_FLAG"] = TEST_FLAG;
    configMap["REBOOT_DEVICE"] = REBOOT_DEVICE;
    configMap["HEARTBEAT_INTERVAL"] = HEARTBEAT_INTERVAL;
    configMap["ACK_NPW_BUFF"] = ACK_NPW_BUFF;
    configMap["NPW_THR_PT1"] = NPW_THR_PT1;
    configMap["NPW_THR_PT2"] = NPW_THR_PT2;
    configMap["NPW_THR_PT3"] = NPW_THR_PT3;
    configMap["NPW_THR_PT4"] = NPW_THR_PT4;
    configMap["SCALING_OFFSET_PT"] = SCALING_OFFSET_PT;
    configMap["SCALING_FACTOR_PT"] = SCALING_FACTOR_PT;
    configMap["FLAG_NPW_SUPPRESS"] = FLAG_NPW_SUPPRESS;
    configMap["NUM_SAMPLES_PT_PERIODIC"] = NUM_SAMPLES_PT_PERIODIC;
    configMap["TIMEZONE_OFFSET"] = TIMEZONE_OFFSET;
}

EdgeDevice::EdgeDevice(const char *confFilePath) {
    LOG(INFO) << "EdgeDevice constructor, confFilePath: " << confFilePath;
    commPtr = nullptr;
    modbusMaster = nullptr;

    keepRunning = true;
    nextHBTimePoint = std::chrono::high_resolution_clock::now();
    applicationStartTime = std::chrono::high_resolution_clock::now();
    initializeRegisterMap();
    heartbeatInterval = registerMap[HEARTBEAT_INTERVAL];
    registerMap[EDGE_START_TIME] =
            std::chrono::duration_cast<std::chrono::seconds>(
                    applicationStartTime.time_since_epoch()).count();
    NpwBuffer::npwTimezoneOffset = registerMap[TIMEZONE_OFFSET];
    saveRegisterMapToFile();

    initializeConfigMap();
    try {
        char *configFileReadBuffer = readConfigFile(confFilePath);
        LOG(INFO) << configFileReadBuffer;

        rapidjson::Document deviceConfigDoc;
        deviceConfigDoc.Parse(configFileReadBuffer);

        rapidjson::Value &edgeDeviceObj = deviceConfigDoc["edge_device"];
        LOG(INFO) << edgeDeviceObj.IsObject();

        deviceId = edgeDeviceObj["device_id"].GetInt();
        deviceName = edgeDeviceObj["device_name"].GetString();
        std::string edgeRoleStr = edgeDeviceObj["role"].GetString();
        if (edgeRoleStr == "bvEdgeDevice") {
            edgeDeviceRole = bvEdgeDevice;
        } else if (edgeRoleStr == "gatewayEdgeDevice") {
            edgeDeviceRole = gatewayEdgeDevice;
        } else if (edgeRoleStr == "stationEdgeDevice") {
            edgeDeviceRole = stationEdgeDevice;
        } else {
            LOG(FATAL) << "Invalid value of edgeDeviceRole in config file";
        }

        rapidjson::Value &communicatorObj = edgeDeviceObj["communicator"];

        std::string communicatorTypeStr = communicatorObj["type"].GetString();

        communicator *commPtr = nullptr;
        if (communicatorTypeStr == "radioCommunicator") {
            commPtr = new RadioCommunicator(this, modbusModeSlave,
                    communicatorObj);
        } else if (communicatorTypeStr == "mqttCommunicator") {
            commPtr = new MqttCommunicator(this, communicatorObj);
        }

        setCommunicator(commPtr);

        if (edgeDeviceRole == gatewayEdgeDevice) {
            rapidjson::Value &modbusMasterObj = edgeDeviceObj["modbus_master"];
            RadioCommunicator *modbusMasterPtr = new RadioCommunicator(this,
                    modbusModeMaster, modbusMasterObj);
            setModbusMaster(modbusMasterPtr);
        } else {    //BV or Station edge devices
            rapidjson::Value &sensorsArray = edgeDeviceObj["sensors"];
            Sensor *sensorPtr = nullptr;
            for (unsigned int i = 0; i < sensorsArray.Size(); i++) {
                if (sensorsArray[i]["sensor_type"] == "PT") {
                    sensorPtr = new PressureSensor(commPtr, this,
                            sensorsArray[i]);
                } else if (sensorsArray[i]["sensor_type"] == "TT") {
                    sensorPtr = new TemperatureSensor(commPtr, this,
                            sensorsArray[i]);
                } else {
                    LOG(FATAL) << "Unknown sensor type in JSON config file";
                    delete configFileReadBuffer;
                    return;
                }
                //enable pushing periodic values for station edge devices,
                //for BV edge devices, periodic values would be read via polling
                sensorPtr->enablePeriodicValues =
                        edgeDeviceRole == stationEdgeDevice;

                addSensor(sensorPtr);
            }
        }
        delete configFileReadBuffer;
    } catch (const std::exception &e) {
        LOG(FATAL) << "Got exception while parsing config,json file: "
                << e.what();
    }
}

bool EdgeDevice::updateRegisterValue(CommandMsg *incomingCommand) {
    if (incomingCommand->getCommand() > UNINITIALIZED_CMD
            && incomingCommand->getCommand() < INVALID_COMMAND) {
        registerMap[incomingCommand->getCommand()] = incomingCommand->getData();
        if (!saveRegisterMapToFile()) {
            LOG(ERROR) << "Unable to save register map to file";
        }
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

/*
 * Receives a command name and value and converts it into a command object
 * to pass it to the overloaded processIncomingCommand function.
 */
void EdgeDevice::processIncomingCommand(std::string registerName, uint32_t data){
    auto confIterator = configMap.find(registerName);
    if (confIterator != configMap.end()) {
        CommandMsg *cmd = new CommandMsg(confIterator->second, data);
        processIncomingCommand(cmd);
    } else {
        LOG(WARNING) << "command not found in map, " << registerName;
//                    command = INVALID_COMMAND;
    }
}

void EdgeDevice::updateTimezoneOffset(CommandMsg *incomingCommand) {
    if (incomingCommand->data > kDcTimezoneOffset.min
            && incomingCommand->data < kDcTimezoneOffset.max) {
        LOG(INFO) << "Setting new timezone offset: "
                << incomingCommand->data;
        NpwBuffer::npwTimezoneOffset = incomingCommand->data;
        updateRegisterValue(incomingCommand);
    } else {
        LOG(WARNING) << "Discarding invalid value of timezone offset: "
                << incomingCommand->data;
    }
}

void EdgeDevice::processIncomingCommand(CommandMsg * incomingCommand){
    if (edgeDeviceRole == gatewayEdgeDevice
            and deviceId != incomingCommand->getDeviceId()) {
        LOG(INFO) << "Gateway Edge device, going to enqueue command for modbus slave.";
        LOG_IF(FATAL, modbusMaster==nullptr);
        if (nullptr == modbusMaster or
                false == modbusMaster->enqueueSlaveCommand(incomingCommand))
        {
            LOG(ERROR) << "Failed to enqueue slave command.";
        }

        if (HEARTBEAT_INTERVAL == incomingCommand->getCommand()) {
            setHeartbeatInterval(incomingCommand);
        }

        return;
    }


    switch (incomingCommand->getCommand()){
    case UNINITIALIZED_CMD:
        LOG(WARNING) << "Uninitialized command received.";
        delete incomingCommand;
        break;
    case REBOOT_DEVICE:
        //just exit the application.
        //In production, the application would run as a systemd service, and it
        //would restart automatically after shutdown.
        if (deviceId == incomingCommand->getData()) {
            LOG(INFO) << "Received reboot command for device: " << deviceId;
            keepRunning = false;
        }
        delete incomingCommand;
        break;
    case HEARTBEAT_INTERVAL:
        setHeartbeatInterval(incomingCommand);
        delete incomingCommand;
        break;
    case NPW_NUM_PACK:
        if (commPtr->setNpwPacketsToBuffer(incomingCommand->getData())) {
            updateRegisterValue(incomingCommand);
        }
        delete incomingCommand;
        break;
    case TIMEZONE_OFFSET:
        updateTimezoneOffset(incomingCommand);
        delete incomingCommand;
        break;
    case MAX_TIME_PERIODIC:
    case MIN_TIME_PERIODIC:
    case ON_CHANG_THSH_PT:
    case ON_CHANG_THSH_TT:
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
    case SCALING_FACTOR_PT:
    case SCALING_OFFSET_PT:
    case FLAG_NPW_SUPPRESS:
    case NUM_SAMPLES_PT_PERIODIC:
    case NPW_SAMPLE_BEFORE:
    case NPW_SAMPLE_AFTER:
        for (Sensor * sensorPtr : sensorsList) {
            sensorPtr->enqueueCommand(incomingCommand);
        }
        delete incomingCommand;
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
    if (hb >= kDcHeartbeatInterval.min and hb <= kDcHeartbeatInterval.max) {
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

bool EdgeDevice::sendMessage(CommDataBuffer * d) {
    if (commPtr == nullptr) {
        LOG(ERROR) << "commPtr not set for Edge Device. Cannot send message.";
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
        int noOfSuccessfulMsgs = 0;
        int noOfFailedMsgs = 0;
        uint64_t timeSpentInComm = 0;

        if (edgeDeviceRole == gatewayEdgeDevice) {
            LOG_IF(FATAL, modbusMaster == nullptr)<<
            "modbusMaster is not nullptr in gatewayEdgeDecice";
            modbusMaster->getCommunicationStats(noOfFailedMsgs, noOfSuccessfulMsgs,
                    timeSpentInComm);

            registerMap[COMM_FAILURE_COUNT] = noOfFailedMsgs;
            registerMap[COMM_SUCCESS_COUNT] = noOfSuccessfulMsgs;
            registerMap[AVERAGE_TRANS_TIME] = noOfSuccessfulMsgs == 0?0:
            (int32_t)timeSpentInComm/noOfSuccessfulMsgs;
        }

        hbPtr = new HeartbeatBuffer(deviceId, registerMap, deviceName);
        nextHBTimePoint = std::chrono::high_resolution_clock::now()
                + std::chrono::seconds(heartbeatInterval);
        DLOG(INFO) << "Creating new heartbeat buffer";
    }
    return hbPtr;
}

void EdgeDevice::initializeRegisterMap() {
    if (loadRegisterMapFromFile()) {
        LOG(INFO) << "Loaded register map from file";
        return;
    }

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
    registerMap[REBOOT_DEVICE] = kDcRebootTime.def;
    registerMap[HEARTBEAT_INTERVAL] = kDcHeartbeatInterval.def;
    registerMap[SCALING_FACTOR_PT] = kDcNPWScaingFactor.def;
    registerMap[SCALING_OFFSET_PT] = kDcNPWScaingOffset.def;
    registerMap[FLAG_NPW_SUPPRESS] = kDcFlagNPWSuppress.def;
    registerMap[NUM_SAMPLES_PT_PERIODIC] = kDcNumSamplesPeriodicAvg.def;
    registerMap[TIMEZONE_OFFSET] = kDcTimezoneOffset.def;
}

bool EdgeDevice::loadRegisterMapFromFile() {
    bool ret = false;
    std::streampos begin, end;
    std::ifstream regMapFile(kRegMapFileName, std::ios::binary);
    if (regMapFile.is_open()) {
        begin = regMapFile.tellg();
        regMapFile.seekg(0, regMapFile.end);
        end = regMapFile.tellg();
        if (end - begin != sizeof(registerMap)) {
            LOG(WARNING) << "Invalid size of data in input file: "
                    << end - begin;
        } else {
            regMapFile.seekg(0, regMapFile.beg);
            regMapFile.read((char*) &registerMap[0], sizeof(registerMap));
            if (regMapFile) {
                LOG(INFO) << "Successfully read register map from file.";
                ret = true;
            } else {
                LOG(INFO)
                        << "Unable to read complete register map, bytes count: "
                        << regMapFile.gcount();
            }
        }
    } else {
        LOG(WARNING) << "Unable to open register map file.";
    }

    regMapFile.close();

    return ret;
}

bool EdgeDevice::saveRegisterMapToFile() {
    std::ofstream regMapFile;
    bool ret = false;
    try {

        regMapFile.open(kRegMapFileName,
                std::fstream::binary | std::fstream::trunc);
        if (regMapFile.good()
                and (regMapFile.rdstate() & std::ifstream::failbit) != 0)
            LOG(ERROR) << "Error opening: " << kRegMapFileName;

        if (not regMapFile.is_open()) {
            std::ios_base::iostate rds = regMapFile.rdstate();
            LOG(ERROR) << "Cannot open register map file: " << rds;
            return false;
        }

        regMapFile.write((char *)&registerMap[0], sizeof(registerMap));
        if (regMapFile.good()) {
            LOG(INFO) << "Register Map saved to file ";
            ret = true;
        } else {
            LOG(ERROR) << "Failed to save register map";
        }

    } catch (const std::exception &e) {
        LOG(ERROR) << " Exception in saveRegisterMapToFile: " << e.what();
        ret = false;
    }

    regMapFile.flush();
    regMapFile.close();

    return ret;
}

bool EdgeDevice::addConfigToConfigMqp(const std::string registerName,
        uint8_t deviceId, CommandRegister cmdReg) {
    if (cmdReg < UNINITIALIZED_CMD or cmdReg >INVALID_COMMAND) {
        LOG(WARNING) << "Unknown command received to add to map: " << cmdReg;
        return false;
    }
    try {
        configMap[registerName] = CommandMsg(cmdReg, 0, deviceId);
    } catch (const std::exception &e) {
        LOG(ERROR) << " Exception in adding configuration to map: " << e.what();
        return false;
    }
    return true;
}
