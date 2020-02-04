/*
 * SerialCommunicator.cpp
 *
 *  Created on: Jan 15, 2020
 *      Author: Fahad Usman
 */

#include <RadioCommunicator.h>

#include <string>

#include "communicator.h"

RadioCommunicator::RadioCommunicator(EdgeDevice * d, const int & slaveAddress):communicator(d) {
    modbusMode = modbusSlave;
    slaveThreadPtr = NULL;
    isConnected = false;
    modbusSlaveAddress = slaveAddress;
}

RadioCommunicator::~RadioCommunicator() {
    // TODO Auto-generated destructor stub
}

void RadioCommunicator::connect() {
    do {
        try {
            sStream.Open("/dev/ttyUSB0");
            isConnected = true;
        }
        catch (const OpenFailed & e) {
            LOG(ERROR) << "The serial port did not open correctly." << e.what();
//            return;
            usleep(2000000);
        }
    } while (not isConnected);

    sStream.SetBaudRate(BaudRate::BAUD_115200);
    sStream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    sStream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
    sStream.SetParity(Parity::PARITY_NONE) ;
    sStream.SetStopBits(StopBits::STOP_BITS_1);
}

void RadioCommunicator::modbusSlaveThread() {
    bool packetStarted = false;
    std::string receiveBuffer = "";
    do {
        while (sStream.rdbuf()->in_avail() == 0) {
            usleep(200000);
            std::cout << "|" << std::flush;
        }

        usleep(1000);
        std::cout << "available: " << sStream.GetNumberOfBytesAvailable()
                << "\t";
        char data_byte;
        while (sStream.IsDataAvailable()) {
            data_byte = sStream.get();
            std::cout << " " << std::hex << int(data_byte);
            if (data_byte == ':') {
                packetStarted = true;
                receiveBuffer = ":";
                LOG(INFO) << "packet started.";
                continue;
            }

            if (packetStarted) {
                if (data_byte == 0x0D) {
                    receiveBuffer += data_byte;
                    data_byte = sStream.get();
                    if (data_byte == 0x0A) {
                        receiveBuffer += data_byte;
                        LOG(INFO) << "packet received\n" << receiveBuffer;
                        if (processIncomingMessage(receiveBuffer.c_str(),
                                receiveBuffer.length())) {
                            LOG(INFO) << "Incoming message processed successfully";
                        }
                    } else {
                        LOG(WARNING) << "Discarding MODBUS message\n"
                                << receiveBuffer;
                    }
                    receiveBuffer = "";
                    packetStarted = false;
                } else if ((data_byte >= '0' and data_byte <= '9')
                        or (data_byte >= 'A' and data_byte <= 'F')
                        or (data_byte >= 'a' and data_byte <= 'f')) {

                    receiveBuffer += data_byte;
                } else {
                    LOG(WARNING)
                            << "invalid character received, discarding buffer\n"
                            << receiveBuffer;
                    receiveBuffer = "";
                    packetStarted = false;
                }
            }
            // Wait a brief period for more data to arrive.
            usleep(1000);
        }
        std::cout << "Done." << std::endl;
    } while (true);
}

void RadioCommunicator::subscribe() {
    //start modbus slave thread.
    modbusSlaveThread();
}

void RadioCommunicator::disconnect() {
    sStream.Close();
    return;
}

void RadioCommunicator::sendMessage(const char * message,
        const unsigned int length) {
    LOG(INFO) << "radio::sendMessage: " << message << "len: " << length;
    return;
}

unsigned char LRC(const char *nData, int wLength) {
    char nLRC = 0; // LRC char initialized

    for (int i = 0; i < wLength; i++)
        nLRC += *(nData+i);

    return (unsigned char)(-nLRC);
} // End: LRC

// send one message from transmit queue as a response for this command.
void RadioCommunicator::transmitMessage(CommDataBuffer* commPtr) {
    unsigned char * serializedMessage = NULL;
    {
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        if (transmitQueue.size() > 0) {
            commPtr = transmitQueue.begin()->second;
            transmitQueue.erase(transmitQueue.begin());
        }
    }
    if (commPtr != NULL) {
        uint64_t currentTime = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        if (0 != commPtr->getExpiryTime()
                && currentTime > commPtr->getExpiryTime()) {
            LOG(WARNING) << "Discarding expired message, id: "
                    << commPtr->getBufferId() << "\tt: "
                    << commPtr->getTimestamp() << "\tExp time: "
                    << commPtr->getExpiryTime();
        } else {
            LOG(INFO) << "Sending Message with t: "
                    << commPtr->getTimestamp();
            int serializedMsgLen = 0;
            serializedMessage = commPtr->serialize(serializedMsgLen); //TODO: Implement serialize function
            if (serializedMessage != nullptr && serializedMsgLen > 0) {
                //                    char * modbusResponse = new char [serializedMsgLen + kModbusResponseHdrLength + 1];
                char* asciiMessage = binaryToHex(serializedMessage,
                        serializedMsgLen);
                if (asciiMessage == nullptr) {
                    LOG(ERROR)
                            << "Unable to convert binary buffer to ascii, discarding message, id: "
                            << commPtr->getBufferId();
                    delete serializedMessage;
                    delete commPtr;
                } else {
                    std::string modbusResponse = ":0103";
                    unsigned char size =
                            (serializedMsgLen < 127) ?
                                    (unsigned char) serializedMsgLen : 0xFF;
                    char * tempHexStr = binaryToHex(&size, 1);
                    modbusResponse.append(tempHexStr);
                    delete tempHexStr;
                    modbusResponse.append(asciiMessage);
                    unsigned char lrc = LRC(modbusResponse.c_str(),
                            modbusResponse.length());
                    tempHexStr = binaryToHex(&lrc, 1);
                    modbusResponse.append(tempHexStr);
                    delete tempHexStr;
                    modbusResponse.append("\r\n");
                    LOG(INFO) << "modbus response len: "
                            << modbusResponse.length() << "\tdata: "
                            << modbusResponse;
                    sendMessage(modbusResponse.c_str(),
                            modbusResponse.length());
                }
                delete serializedMessage;
            }
        }
    }
}

bool RadioCommunicator::processIncomingMessage(const char * message,
        const int length) {
    LOG(INFO) << "radio::processIncomingMessage: " << message << "len: "
            << length;
//    TODO: Verify checksum
    if (strlen(message) < 16 or length < 16) {
        LOG(WARNING) << "Incoming message is too short, ignoring";
        return false;
    }
    ModbusMessage modbusMsg = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    try {
        std::string temp = "";
        temp[0] = message[1];
        temp[1] = message[2];
        temp[2] = '\0';
        modbusMsg.slaveAddress = std::stoi(temp, 0, 16); //convert from hex (base16) encoded string to integer.

        if (modbusMsg.slaveAddress != modbusSlaveAddress) {
            LOG(INFO) << "Discarding message, slave id ("
                    << modbusMsg.slaveAddress << ")not matched";
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
        temp[3] = message[12];                                                                                   temp[4] = '\0';
        modbusMsg.writeData = std::stoi(temp, 0, 16);

        temp[0] = message[13];
        temp[1] = message[14];
//        temp[2] = message[15];
//        temp[3] = message[16];
        temp[3] = '\0';
        modbusMsg.errorCheck = std::stoi(temp, 0, 16);
        LOG(INFO) << std::hex << "Processed message, \nslaveAddress: " << modbusMsg.slaveAddress
            << "\nfunctionCode: " << modbusMsg.functionCode
            << "\nregisterAddress: " << modbusMsg.registerAddress
            << "\nwriteData: " << modbusMsg.writeData
            << "\nerrorCheck: " << modbusMsg.errorCheck;
    }
    catch (const std::exception &e) {
        LOG(WARNING) << "Error parsing incoming message" << e.what();
        return false;
    }

    CommandMsg * cmd = NULL;
    CommDataBuffer * commPtr = NULL;

    switch (modbusMsg.functionCode) {
    case WRITE_SINGLE_REGISTER:
        cmd = new CommandMsg(UNINITIALIZED_CMD, modbusMsg.writeData);
        if (edgeDevicePtr == NULL) {
            LOG(FATAL) << "edgeDevicePtr is NULL";
            return false;
        }
        edgeDevicePtr->processIncomingCommand(cmd);
        //after successful processing, echo back the received message (as prescribed by modbus protocol.
        sStream.write(message, length);
        break;
    case READ_HOLDING_REGISTERS:
        // send one message from transmit queue as a response for this command.
        transmitMessage(commPtr);
        break;
    default:
        LOG(WARNING) << "Unhandled modbus function code.";
    }

    return true;
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

    std::stringstream hexStream;
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
