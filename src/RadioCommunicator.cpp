/*
 * SerialCommunicator.cpp
 *
 *  Created on: Jan 15, 2020
 *      Author: Fahad Usman
 */

#include <RadioCommunicator.h>

#include <string>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "HeartbeatBuffer.h"
std::map<int, std::string> HeartbeatBuffer::deviceNameMap;

void RadioCommunicator::initializeVariables(ModbusModes mode,
        const std::string &radioPort, const int &slaveAddress) {
    slaveThreadPtr = nullptr;
    masterThreadPtr = nullptr;
    isModbusStreamConnected = false;
    masterThreadDone = true;
    slaveThreadDone = true;
    modbusMode = mode;
    radioSerialPort = radioPort;
    modbusSlaveAddress = slaveAddress;
    commandMessageRetryLimit = 3;
}

RadioCommunicator::RadioCommunicator(EdgeDevice *d, ModbusModes mode,
        const rapidjson::Value &communicatorObj) :
        communicator(d, mode == modbusModeSlave) { //enable disk persistence for modbus slave only
    int slaveAddress = 0;
    std::string radioPort = "";
    baudRate = BaudRate::BAUD_115200;
    try {
        radioPort = communicatorObj["port"].GetString();
        //TODO: baud_rate, response_timeout, poll_interval, transmission_timeout
        int32_t bRate = communicatorObj["baud_rate"].GetInt();
        switch (bRate) {
        case 9600:
            baudRate = BaudRate::BAUD_9600;
            break;
        case 38400:
            baudRate = BaudRate::BAUD_38400;
            break;
        case 115200:
            baudRate = BaudRate::BAUD_115200;
            break;
        default:
            baudRate = BaudRate::BAUD_115200;
            LOG(FATAL) << "Unsupported baud rate in config file: " << bRate;

        }

        modbusResponseTimeout = std::chrono::milliseconds(1)
                * communicatorObj["response_timeout"].GetInt();

        modbusMasterPollInterval = std::chrono::milliseconds(1)
                * communicatorObj["poll_interval"].GetInt();

        modbusTransmissionTimeout = std::chrono::milliseconds(1)
                * communicatorObj["transmission_timeout"].GetInt();

        if (mode == modbusModeMaster) {
            const rapidjson::Value & slavesList = communicatorObj["slave_list"];
            LOG_IF(FATAL, not slavesList.IsArray()) << "slave_ids is not a JSON Array";
            for (unsigned int i = 0; i < slavesList.Size(); i++) {
                uint8_t deviceId = (uint8_t) (slavesList[i]["device_id"].GetInt());
                HeartbeatBuffer::deviceNameMap[deviceId] = slavesList[i]["device_name"].GetString();
                addModbusSlave(deviceId,
                        slavesList[i]["device_name"].GetString());
                const rapidjson::Value & sensorsListObj = slavesList[i]["sensors"];
                for (unsigned int j = 0; j < sensorsListObj.Size(); j++) {
                    LOG(INFO) << "sensor type: "
                            << sensorsListObj[j]["sensor_type"].GetString() << " sensor id: "
                            << sensorsListObj[j]["sensor_id"].GetString();
//                    add new configuration in map (register name = NPW_THR_ + sensor_id, device_id, NPW_THR_PT1+j
                    if (std::string(sensorsListObj[j]["sensor_type"].GetString()) == "PT") {
                        LOG(INFO) << "Going to add new config to map, Key: " << "NPW_THR_"
                                << sensorsListObj[j]["sensor_id"].GetString()
                                << "\tValue: " << (CommandRegister)((int)NPW_THR_PT1 + j);
                        edgeDevicePtr->addConfigToConfigMqp(
                            std::string("NPW_THR_")
                                    + sensorsListObj[j]["sensor_id"].GetString(),
                            deviceId, (CommandRegister)((int)NPW_THR_PT1 + j));
                    }
                }

            }
        } else {
            slaveAddress = communicatorObj["slave_address"].GetInt();
        }
    } catch (const std::exception &e) {
        LOG(FATAL) << "Got exception while parsing config,json file: "
                << e.what();
    }

    initializeVariables(mode, radioPort, slaveAddress);
    connect();
    if (modbusMode == modbusModeSlave) {
        subscribe();
    } else if (modbusMode == modbusModeMaster) {
        startModbusMaster();
    }
}

RadioCommunicator::~RadioCommunicator() {
    slaveThreadDone = true;
    masterThreadDone = true;
    if (slaveThreadPtr != nullptr) {
        LOG(INFO) << "Waiting for Modbus slave thread to end.";
        slaveThreadPtr->join();
    }

    if (masterThreadPtr != nullptr) {
        LOG(INFO) << "Waiting for Modbus master thread to end.";
        masterThreadPtr->join();
    }

    disconnect();

    for (ModbusSlave * modbusSlavePtr:modbusSlavesList) {
        delete modbusSlavePtr;
    }
    modbusSlavesList.clear();
}

void RadioCommunicator::connect() {
    if (isModbusStreamConnected) {
        modbusStream.Close();
        isModbusStreamConnected = false;
    }
    while (not isModbusStreamConnected) {
        try {
            modbusStream.Open(radioSerialPort);
            isModbusStreamConnected = true;
        }
        catch (const OpenFailed & e) {
            LOG(ERROR) << "The serial port (" << radioSerialPort
                    << ")did not open correctly." << e.what();
            isModbusStreamConnected = false;
            usleep(2000000);
        }
    }

    modbusStream.SetBaudRate(baudRate);
    modbusStream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    modbusStream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
    modbusStream.SetParity(Parity::PARITY_NONE) ;
    modbusStream.SetStopBits(StopBits::STOP_BITS_1);
}

bool RadioCommunicator::sendQueuedCommand() {
    CommandMsg* cmdPtr = getQueuedSlaveCommand();
    if (cmdPtr != nullptr) {
        bool slaveIdFound = false;
        for (auto x:modbusSlavesList) {
            if (x->slaveId == cmdPtr->getDeviceId()) {
                slaveIdFound = true;
                break;
            }
        }

        if (not slaveIdFound) {
            LOG(WARNING) << "Discarding command, slaveId not found in modbusSlavesList.";
            popQueuedSlaveCommand();
            delete cmdPtr;
            return false;
        }

        if (sendModbusCommand(cmdPtr->getDeviceId(), cmdPtr->getCommand(),
                cmdPtr->getData())) {
            popQueuedSlaveCommand();
            delete cmdPtr;
            cmdPtr = nullptr;
            return true;
        }
        else {
            LOG(WARNING) << "Failed to send modbus command";
        }
        if (++(cmdPtr->retryCount) >= commandMessageRetryLimit) {
            LOG(WARNING) << "Discarding command, retry limit reached.";
            popQueuedSlaveCommand();
            delete cmdPtr;
        }
    }
    return false;
}

void RadioCommunicator::modbusMasterThread() {
    std::string readRegistersCommand = ":01030000000280\r\n";
    auto slaveIt = modbusSlavesList.begin();
    std::string receiveBuffer = "";
    std::chrono::time_point<std::chrono::high_resolution_clock>
        t1 = std::chrono::high_resolution_clock::now(),
        t2 = std::chrono::high_resolution_clock::now();
    while (not masterThreadDone) {
        std::this_thread::sleep_for(modbusMasterPollInterval);
        t1 = std::chrono::high_resolution_clock::now();
        if (sendQueuedCommand()) {
            continue;
        }

        try {
            if (slaveIt == modbusSlavesList.end()) {
                slaveIt = modbusSlavesList.begin();
            }
            modbusSlaveAddress = (*slaveIt)->slaveId;
            readRegistersCommand = (*slaveIt++)->modbus0x03Command;
            LOG(INFO) << "modbusMaster polling slave for data, "
                    << "modbusSlaveAddress: " << (int) modbusSlaveAddress;
            modbusStream.write(readRegistersCommand.c_str(),
                    readRegistersCommand.length());
            modbusStream.DrainWriteBuffer();
        } catch (const std::exception & e) {
            LOG(ERROR) << "Exception: " << e.what();
            connect();
        }
        //wait for response from the slave.
        std::chrono::time_point<std::chrono::high_resolution_clock> expTime =
                std::chrono::high_resolution_clock::now()
                        + modbusResponseTimeout;

        receiveBuffer = "";
        bool responseSuccess = false;
        while (std::chrono::high_resolution_clock::now() < expTime) {
            if (modbusStream.rdbuf()->in_avail() == 0) {
                usleep(50000); //TODO: remove hard coded value
            } else if (receiveModbusAsciiMessage(receiveBuffer, expTime)) {
                if (processIncomingMessage(receiveBuffer.c_str(),
                        receiveBuffer.length())) {
                    LOG(INFO) << "Incoming message processed successfully";
                    responseSuccess = true;
                    break;
                } else {
                    LOG(WARNING) << "message processing failed, msg: "
                            << receiveBuffer;
                }
            }
        }

        t2 = std::chrono::high_resolution_clock::now();
        if (responseSuccess) {
            addCommunicationTime(t2-t1);
        } else {
            incFailedTransferCount();
        }
    }
}

bool RadioCommunicator::receiveModbusAsciiMessage(std::string& receiveBuffer,
        std::chrono::time_point<std::chrono::high_resolution_clock> expTime) {
    LOG(INFO) << "data available, number of bytes:"
            << modbusStream.GetNumberOfBytesAvailable();
    char data_byte = 0;
    bool packetStarted = false;
    while (modbusStream.IsDataAvailable()) {
        data_byte = modbusStream.get();
        if (data_byte == ':') {
            packetStarted = true;
            receiveBuffer = ":";
            LOG(INFO) << "packet started.";
            continue;
        }
        if (packetStarted) {
            if (data_byte == 0x0D) {
                receiveBuffer += data_byte;
                data_byte = modbusStream.get();
                if (data_byte == 0x0A) {
                    receiveBuffer += data_byte;
                    LOG(INFO) << "packet received\n" << receiveBuffer;
                    return true;
                } else {
                    LOG(WARNING) << "Discarding MODBUS message\n"
                            << receiveBuffer;
                    return false;
                }
                receiveBuffer = "";
                packetStarted = false;
            } else if ((data_byte >= '0' && data_byte <= '9')
                    || (data_byte >= 'A' && data_byte <= 'F')
                    || (data_byte >= 'a' && data_byte <= 'f')) {
                receiveBuffer += data_byte;
            } else {
                LOG(WARNING)
                        << "invalid character received, discarding buffer\n"
                        << receiveBuffer;
                receiveBuffer = "";
                packetStarted = false;
            }
            while (not modbusStream.IsDataAvailable()
                    and std::chrono::high_resolution_clock::now() < expTime) {
                // Wait a brief period for more data to arrive.
                LOG(INFO) << "packet started, but IsDataAvailable == false, waiting briefly. receiveBuffer.len: " << receiveBuffer.length();
                usleep(50000);
            }
        }
    }
    return false;
}

void RadioCommunicator::modbusSlaveThread() {
    std::string receiveBuffer = "";
    while (not slaveThreadDone) {
        LOG(INFO) << "waiting for command from master";
        while (modbusStream.rdbuf()->in_avail() == 0) {
            usleep(200000);
        }

        usleep(1000);
        if (receiveModbusAsciiMessage(receiveBuffer,
                std::chrono::high_resolution_clock::now()
                        + modbusTransmissionTimeout)) {
            if (processIncomingMessage(receiveBuffer.c_str(),
                    receiveBuffer.length())) {
                LOG(INFO) << "Incoming message processed successfully";
            } else {
                LOG(WARNING) << "message processing failed, msg: " << receiveBuffer;
            }
        }
    }
}

void RadioCommunicator::subscribe() {
    //start modbus slave thread.
    slaveThreadDone = false;
    slaveThreadPtr = new std::thread(&RadioCommunicator::modbusSlaveThread, this);
    LOG(INFO) << "modbusSlaveThread launched";
}

void RadioCommunicator::startModbusMaster() {
    //start modbus master thread.
    masterThreadDone = false;
    masterThreadPtr= new std::thread(&RadioCommunicator::modbusMasterThread, this);
    LOG(INFO) << "modbusMasterThread launched";
}

void RadioCommunicator::disconnect() {
    //TODO: Discard queued messages.
    modbusStream.Close();
    isModbusStreamConnected = false;
    return;
}

void RadioCommunicator::sendMessage(const char * message,
        const unsigned int length) {
    LOG(INFO) << "radio::sendMessage, len: " << length;
    if (not isModbusStreamConnected) {
        connect(); //block till a connection is established
    }
    try {
        modbusStream.write(message, length);
        modbusStream.DrainWriteBuffer();
    } catch (const std::exception & e) {
        LOG(ERROR) << "Got exception in sending serial message: " << e.what();
        modbusStream.Close();
        isModbusStreamConnected = false;
    }

    return;
}

uint8_t LRC(const char *nData, int wLength) {
    char nLRC = 0; // LRC char initialized

    for (int i = 0; i < wLength; i++)
        nLRC += nData[i];

    return (uint8_t)(-nLRC);
} // End: LRC

/* Convert binary serialized buffer to Modbus Ascii message to be sent as a
 * response to READ_REGISTERS command (0x03). Only deviation from the standard
 * Modbus response is that Byte Count is stored in two bytes
 */
std::string RadioCommunicator::binaryToModbusAsciiMessage(int serializedMsgLen,
        unsigned char* binaryData) {
    std::string modbusResponse = ":";
    char* tempHexStr = binaryToHex((unsigned char *)&modbusSlaveAddress, 1);
    modbusResponse.append(tempHexStr);
    delete tempHexStr;
    modbusResponse.append("03");
    uint16_t size = serializedMsgLen;
    tempHexStr = binaryToHex((unsigned char *)&size, 2);
    modbusResponse.append(tempHexStr);
    delete tempHexStr;
    tempHexStr = binaryToHex(binaryData, serializedMsgLen);
    modbusResponse.append(tempHexStr);
    delete tempHexStr;
    unsigned char lrc = LRC(modbusResponse.c_str(), modbusResponse.length());
    tempHexStr = binaryToHex(&lrc, 1);
    modbusResponse.append(tempHexStr);
    delete tempHexStr;
    modbusResponse.append("\r\n");
    return modbusResponse;
}

// send one message from transmit queue.
void RadioCommunicator::transmitMessage() {
    CommDataBuffer* commPtr = getQueuedMessage();
    unsigned char * serializedMessage = nullptr;

    if (commPtr == nullptr) {
        commPtr = edgeDevicePtr->getHeartBeat();
    }

    if (commPtr == nullptr) {
        commPtr = edgeDevicePtr->getPeriodicSensorValue();
        LOG_IF(INFO, (commPtr != nullptr))
            << "no message in queue, sending current periodic value";
    }

    if (commPtr != nullptr) {
        uint64_t currentTime = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        if (0 != commPtr->getExpiryTime()
                && currentTime > commPtr->getExpiryTime()) {
            LOG(WARNING) << "Discarding expired message, id: "
                    << commPtr->getBufferId() << "\tt: "
                    << commPtr->getTimestamp() << "\tExp time: "
                    << commPtr->getExpiryTime();
            this->removeMessageFromQueue(commPtr->getBufferId());
        } else {
            LOG(INFO) << "Sending Message with t: "
                    << commPtr->getTimestamp();
            int serializedMsgLen = 0;
            serializedMessage = commPtr->serialize(serializedMsgLen);
            if (serializedMessage != nullptr && serializedMsgLen > 0) {
//                char* asciiMessage = binaryToHex(serializedMessage,
//                        serializedMsgLen);
//                if (asciiMessage == nullptr) {
//                    LOG(ERROR)
//                            << "Unable to convert binary buffer to ascii, discarding message, id: "
//                            << commPtr->getBufferId();
//                    delete serializedMessage;
//                    delete commPtr;
//                } else {
                std::string modbusResponse = binaryToModbusAsciiMessage(
                        serializedMsgLen, serializedMessage);
                LOG(INFO) << "modbus response len: " << modbusResponse.length();

                sendMessage(modbusResponse.c_str(),
                        modbusResponse.length());
//                }
                delete serializedMessage;
            }
        }
    }
}

bool RadioCommunicator::parseModbusCommand(ModbusMessage & modbusMsg,
        const char * message, const int & msgLen) {

    if (msgLen < 16) {
        LOG(WARNING) << "Modbus command length too shot. Length: " << msgLen
                << "\nMessage: " << message;
        return false;
    }

    try {
        std::string temp = "";

        temp[0] = message[13];
        temp[1] = message[14];
        temp[2] = '\0';
        modbusMsg.errorCheck = std::stoi(temp, 0, 16);

        unsigned char lrc = LRC(message, 13);
        if (lrc != modbusMsg.errorCheck) {
            LOG(WARNING) << "LRC mismatch, errorCheck: " << std::hex
                    << (int) modbusMsg.errorCheck << "\tLRC: " << (int) lrc;
            return false;
        }

        temp[0] = message[1];
        temp[1] = message[2];
        temp[2] = '\0';
        modbusMsg.slaveAddress = std::stoi(temp, 0, 16); //convert from hex (base16) encoded string to integer.

        if (modbusMsg.slaveAddress != modbusSlaveAddress) {
            LOG(INFO) << "Discarding message, slave id ("
                    << (int) modbusMsg.slaveAddress << ") not matched";
            return false;
        }

        temp[0] = message[3];
        temp[1] = message[4];
        temp[2] = '\0';
        modbusMsg.functionCode = std::stoi(temp, 0, 16);

        temp[0] = message[5];
        temp[1] = message[6];
        temp[2] = message[7];
        temp[3] = message[8];
        temp[4] = '\0';
        modbusMsg.registerAddress = std::stoi(temp, 0, 16);

        temp[0] = message[9];
        temp[1] = message[10];
        temp[2] = message[11];
        temp[3] = message[12];
        temp[4] = '\0';
        if (modbusMsg.functionCode == READ_HOLDING_REGISTERS) {
            modbusMsg.quantityOfRegisters = std::stoi(temp, 0, 16);
        } else {
            modbusMsg.writeData = std::stoi(temp, 0, 16);
        }

        modbusMsg.print();
    } catch (const std::exception &e) {
        LOG(WARNING) << "Error parsing Modbus command message: " << e.what();
        return false;
    }

    return true;
}
bool RadioCommunicator::parseModbusResponse(ModbusMessage & modbusMsg,
        const char * message, const int & msgLen) {
    try {
        std::string temp = "";
        temp[0] = message[1];
        temp[1] = message[2];
        temp[2] = '\0';
        modbusMsg.slaveAddress = std::stoi(temp, 0, 16); //convert from hex (base16) encoded string to integer.

        if (modbusMsg.slaveAddress != modbusSlaveAddress) {
            LOG(INFO) << "Discarding message, slave id ("
                    << (int) modbusMsg.slaveAddress << ") not matched with "
                    << (int) modbusSlaveAddress;
            return false;
        }

        temp[0] = message[3];
        temp[1] = message[4];
        temp[2] = '\0';
        modbusMsg.functionCode = std::stoi(temp, 0, 16);

        if (modbusMsg.functionCode == READ_HOLDING_REGISTERS) {
            temp[0] = message[7];
            temp[1] = message[8];
            temp[2] = message[5];
            temp[3] = message[6];
            temp[4] = '\0';
            modbusMsg.byteCount = std::stoi(temp, 0, 16);

            if (modbusMsg.byteCount > kModbusMaxPacketLen) {
                LOG(ERROR) << "Invalid modbus byteCount: " << modbusMsg.byteCount;
                return false;
            }

            const int dataStart = 9;
            if (modbusMsg.byteCount * 2 + dataStart + 5/*trailer*/ < msgLen) {
                LOG(WARNING) << "Message length too short, discarding message";
                return false;
            }
            //  TODO: Verify error check
            int dataLength = modbusMsg.byteCount;
            modbusMsg.data =
                    new (std::nothrow) unsigned char [dataLength];

            if (modbusMsg.data == nullptr) {
                LOG(ERROR) << "Unable to allocate memory for data buffer.";
                return false;
            }

            for (int i = 0; i < dataLength; i++) {
                temp[0] = message[dataStart + (2 * i)];
                temp[1] = message[dataStart + (2 * i) + 1];
                temp[2] = '\0';
                modbusMsg.data[i] = (unsigned char) std::stoi(temp, 0, 16);
            }

            temp[0] = message[dataStart + (2 * dataLength)];
            temp[1] = message[dataStart + (2 * dataLength) + 1];
            temp[2] = '\0';
            modbusMsg.errorCheck = std::stoi(temp, 0, 16);
        }
        else if (modbusMsg.functionCode == WRITE_SINGLE_REGISTER) {
            temp[0] = message[5];
            temp[1] = message[6];
            temp[2] = message[7];
            temp[3] = message[8];
            temp[4] = '\0';
            modbusMsg.registerAddress = std::stoi(temp, 0, 16);

            temp[0] = message[9];
            temp[1] = message[10];
            temp[2] = message[11];
            temp[3] = message[12];
            temp[4] = '\0';
            modbusMsg.writeData = std::stoi(temp, 0, 16);

            temp[0] = message[13];
            temp[1] = message[14];
            temp[3] = '\0';
            modbusMsg.errorCheck = std::stoi(temp, 0, 16);

        }

    } catch (const std::exception &e) {
        LOG(WARNING) << "Error parsing Modbus response message: " << e.what();
        return false;
    }


    return true;
}

bool RadioCommunicator::processIncomingMessage(const char * message,
        const int & length) {
    LOG(INFO) << "radio::processIncomingMessage: " << message << "len: "
            << length;
//    TODO: Verify checksum
    if (strlen(message) < 16 or length < 16) {
        LOG(WARNING) << "Incoming message is too short, ignoring";
        return false;
    }

    ModbusMessage modbusMsg = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool ret = false;

    if (modbusMode == modbusModeMaster) {
        CommDataBuffer * receivedData = nullptr;
        if (parseModbusResponse(modbusMsg, message, length)) {
            if (modbusMsg.functionCode != 0x03) {
                LOG(WARNING) << "Invalid fn code in modbus response. "
                        << (int) modbusMsg.functionCode;
                return false;
            }
            if (modbusMsg.data == nullptr) {
                LOG(ERROR) << "modbusMsg.data is not initialized";
                return false;
            }

            if ((BufferType)modbusMsg.data[0] == buffTypeNpwBuffer) {
                receivedData = new NpwBuffer();
            } else if ((BufferType)modbusMsg.data[0] == buffTypePeriodicValue) {
                receivedData = new PeriodicValue(0, 0, "", 0);
            } else if ((BufferType)modbusMsg.data[0] == buffTypeHeartBeat) {
                receivedData = new HeartbeatBuffer();
            } else {
                LOG(ERROR) << "Unknown buffer type, processIncomingMessage. "
                        << int(modbusMsg.data[0]);
                return false;
            }
        } else {
            LOG(WARNING) << "failed to parse modbus response";
            return false;
        }

        if (receivedData->deserialize(modbusMsg.data, modbusMsg.byteCount)) {
            LOG(INFO)
                    << "Received Buffer successfully deserailzed, timestamp: "
                    << receivedData->getTimestamp();
            if (edgeDevicePtr->sendMessage(receivedData) == 1) {
                if (modbusMsg.data[0] == (unsigned char) buffTypeNpwBuffer) {
                    //Send acknowledgment.
                    sendModbusCommand(modbusMsg.slaveAddress, ACK_NPW_BUFF,
                            receivedData->getBufferId());
                }
                ret = true;
            } else {
                delete receivedData;
                LOG(WARNING) << "sendMessage failed, discarding received data";
            }
        } else {
            LOG(ERROR) << "Failed to deserialize received buffer";
            delete receivedData;
            receivedData = nullptr;
            ret = false;
        }

    } else { /* for modbusSlave */
        ret = parseModbusCommand(modbusMsg, message, length);
        if (not ret) {
            LOG(WARNING) << "parseModbusCommand failed.";
            return ret;
        }
        CommandMsg * cmd = nullptr;

        switch (modbusMsg.functionCode) {
        case WRITE_SINGLE_REGISTER:
            //TODO: Handle commands specific to communicator
            if (edgeDevicePtr == NULL) {
                LOG(FATAL) << "edgeDevicePtr is NULL";
                return false;
            }
            cmd = new CommandMsg(CommandRegister(modbusMsg.registerAddress),
                    modbusMsg.writeData, modbusMsg.slaveAddress);
            edgeDevicePtr->processIncomingCommand(cmd);
            //after successful processing, echo back the received message (as prescribed by modbus protocol.
            LOG(INFO) << "Writing back same message to ack: " << message;
            sendMessage(message, length);
            break;
        case READ_HOLDING_REGISTERS:
                transmitMessage();
            break;
        default:
            LOG(WARNING) << "Unhandled modbus function code.";
        }
    }

    return ret;
}

void RadioCommunicator::sendQueuedMessagesThread() {
    LOG(INFO) << "radio::sendQueuedMessagesThread";
    return;
}

/*
 * takes a binary buffer and it's length and converts it into a base-16
 * encoded hex-string (null terminated ASCII string).
 * Returns newly allocated null terminated string in case of success
 * and nullptr in case of failure.
 */
char * binaryToHex(const unsigned char * binBuffer, const int & len){

    char * hexString = new (std::nothrow) char [2 * len +1];
    if (hexString == nullptr) {
        LOG(WARNING) << "Cannot allocate memory for hex string";
        return nullptr;
    }

    for (int i = 0; i < len; i++) {
        sprintf(&(hexString[2*i]), "%02X", binBuffer[i]);
    }
    hexString[2*len] = '\0';
    return hexString;
}

/* Takes a base-16 encoded hex-string (null terminated ASCII string) and attempts
 * to convert into a binary buffer,
 * returns a dynamically allocated binary buffer and sets binaryBuffLen reference
 * to the length of binary buffer in case of success, otherwise returns nullptr.
 */
unsigned char * hexToBinary(const char * hexString, int & binaryBuffLen){
    int hexStringLength = 0;
    unsigned char * binaryBuffer = nullptr;
    try {
        hexStringLength = strlen(hexString);
    } catch (const std::exception & e) {
        LOG(ERROR) << "got exception in strlen" << e.what();
        binaryBuffLen = 0;
        return nullptr;
    }

    if ((hexStringLength % 2 != 0) or (hexStringLength < 2)) {
        LOG(ERROR) << "Invalid length of Hex String: ";
        binaryBuffLen = 0;
        return nullptr;
    }

    try {
        binaryBuffLen = (hexStringLength) / 2;
        binaryBuffer = new (std::nothrow) unsigned char[binaryBuffLen];
        if (binaryBuffer == nullptr) {
            LOG(WARNING) << "Unable to allocate memory for binary buffer.";
        }

        char tempHex[3] = { 0, 0, 0 };
        for (int i = 0; i < binaryBuffLen; i++) {
            tempHex[0] = hexString[i * 2];
            tempHex[1] = hexString[i * 2 + 1];
            binaryBuffer[i] = strtol(tempHex, nullptr, 16);
        }
        return binaryBuffer;
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception creating binary buffer. " << e.what();
    }
    if (binaryBuffer != nullptr) {
        delete binaryBuffer;
    }
    binaryBuffLen = 0;
    return nullptr;
}

bool RadioCommunicator::addModbusSlave(uint8_t sId, std::string slaveName) {
    ModbusSlave * mbSlave = new ModbusSlave;
    mbSlave->slaveId = sId;
    mbSlave->slaveName = slaveName;
    strncpy(mbSlave->modbus0x03Command, ":010300000002FA\r\n\0", 18);
    sprintf(mbSlave->modbus0x03Command+1, "%02X", sId);
    mbSlave->modbus0x03Command[3] = '0';
    uint8_t lrc = LRC(mbSlave->modbus0x03Command, 13);
    sprintf(mbSlave->modbus0x03Command+13, "%02X", lrc);
    mbSlave->modbus0x03Command[15] = '\r';
    LOG(INFO) << "addModbusSlave, sId: " << mbSlave->slaveId
            << "\t0x03Command: " << mbSlave->modbus0x03Command;
    modbusSlavesList.push_back(mbSlave);
    return true;
}

bool RadioCommunicator::sendModbusCommand(uint8_t slaveAddress,
        CommandRegister regAddress, uint16_t value) {

    LOG(INFO) << "sendModbusCommand(slaveAddress: " << std::hex
            << (unsigned int) slaveAddress << ", regAddress: " << regAddress
            << ", value: " << value;

    bool ret = false;
    std::string modbusCommand = ":";

    char* tempHexStr = binaryToHex((unsigned char *)&slaveAddress, sizeof(slaveAddress));
    modbusCommand.append(tempHexStr);
    delete tempHexStr;

    modbusCommand.append("06"); //Function code (write single register)

    uint16_t registerAddress = (uint16_t) regAddress;
    tempHexStr = binaryToHex((unsigned char *)&registerAddress, sizeof(registerAddress));
    modbusCommand.append(tempHexStr+2, 2);
    modbusCommand.append(tempHexStr, 2);
    delete tempHexStr;

    tempHexStr = binaryToHex((unsigned char *)&value, sizeof(value));
    modbusCommand.append(tempHexStr+2, 2);
    modbusCommand.append(tempHexStr, 2);
    delete tempHexStr;

    uint8_t lrc = LRC(modbusCommand.c_str(), modbusCommand.length());
    tempHexStr = binaryToHex((unsigned char *)&lrc, sizeof(lrc));
    modbusCommand.append(tempHexStr);
    delete tempHexStr;

    modbusCommand.append("\r\n\0");

    sendMessage(modbusCommand.c_str(), modbusCommand.length());

    std::chrono::time_point<std::chrono::high_resolution_clock> expTime =
            std::chrono::high_resolution_clock::now()
                    + modbusResponseTimeout;

    std::string receiveBuffer = "";
    while (std::chrono::high_resolution_clock::now() < expTime) {
        if (modbusStream.rdbuf()->in_avail() == 0) {
            usleep(50000);
        } else if (receiveModbusAsciiMessage(receiveBuffer,
                std::chrono::high_resolution_clock::now()
                        + modbusTransmissionTimeout)) {
            if (receiveBuffer == modbusCommand) {
                LOG(INFO) << "got response for command: " << receiveBuffer;
                return true;
            } else {
                LOG(WARNING) << "response: " << receiveBuffer <<
                        " did not match command: " << modbusCommand;
                return false;
            }
        }
    }
    LOG(INFO) << "Modbus slave response timed out.";

    return ret;
}

bool RadioCommunicator::enqueueSlaveCommand(CommandMsg * cmd) {
    //TODO: If slaveId is not in modbusSlavesList, then discard the command.
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (cmd->getDeviceId() == 0) {
            for (auto slave : modbusSlavesList) {
                CommandMsg *c = new CommandMsg(cmd->getCommand(),
                        cmd->getData(), slave->slaveId);
                slaveCommandQueue.push(c);
            }
            delete cmd;
        } else {
            slaveCommandQueue.push(cmd);
        }
        return true;
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }
    return false;
}

CommandMsg * RadioCommunicator::getQueuedSlaveCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (slaveCommandQueue.empty()) {
            return nullptr;
        } else {
            return slaveCommandQueue.front();
        }
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }
    return nullptr;
}

/*
 * Pop one slave command from the queue, but the caller would be responsible
 * for destroying the dynamically allocated command object.
 */
bool RadioCommunicator::popQueuedSlaveCommand() {
    try {
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (slaveCommandQueue.empty()) {
            return false;
        } else {
            slaveCommandQueue.pop();
            return true;
        }
    } catch (std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }
    return false;
}
