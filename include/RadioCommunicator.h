/*
 * SerialCommunicator.h
 *
 *  Created on: Jan 15, 2020
 *      Author: Fahad Usman
 */

#ifndef RADIOCOMMUNICATOR_H_
#define RADIOCOMMUNICATOR_H_

#include <libserial/SerialStream.h>
#include <string>
#include <thread>
#include "rapidjson/document.h"

#include "communicator.h"
#include "EdgeDevice.h"

using namespace LibSerial;

enum ModbusModes {
    modbusModeMaster, modbusModeSlave
};

enum ModbusState {
    waiting,
    packetStarted,

};

enum ModbusFnCodes {
    READ_COIL_STATUS = 1,
    READ_INPUT_STATUS = 2,
    READ_HOLDING_REGISTERS = 3,
    READ_INPUT_REGISTERS = 4,
    WRITE_SINGLE_COIL = 5,
    WRITE_SINGLE_REGISTER = 6,
    WRITE_MULTIPLE_COILS = 15,
    WRITE_MULTIPLE_REGISTERS = 16,
};

struct ModbusMessage {
    uint8_t slaveAddress;
    uint8_t  functionCode;
    uint16_t startAddress;
    uint16_t quantityOfRegisters;
    uint16_t byteCount;
    unsigned char * data;
    uint16_t registerAddress;
    uint16_t writeData;
    uint8_t errorCheck;

    void print () {
        LOG(INFO) << std::hex << "Processed message, \n"
            << "slaveAddress: " << (int)slaveAddress
            << "\nfunctionCode: " << (int)functionCode
            << "\nregisterAddress: " << registerAddress
            << "\nquantityOfRegisters: " << quantityOfRegisters
            << "\nwriteData: " << writeData
            << "\nerrorCheck: " << (int)errorCheck;
    }
    ~ModbusMessage(){
        if (data != nullptr) {
            delete data;
        }
    }
};

struct ModbusSlave {
    uint16_t slaveId;
    char modbus0x03Command[18]; //e.g. ":010300000002FA\r\n\0"
    std::string slaveName;
};

const unsigned int kModbusResponseHdrLength = 11;
const unsigned int kModbusMaxPacketLen = 2500;

class RadioCommunicator: public communicator {
protected:
    SerialStream modbusStream;
    std::string readBuffer;
    ModbusModes modbusMode;
    void modbusSlaveThread();
    void modbusMasterThread();
    std::thread * slaveThreadPtr;
    std::thread * masterThreadPtr;
    bool isModbusStreamConnected;
    bool parseModbusCommand(ModbusMessage & modbusMsg, const char * receivedMsg,
            const int & msgLen);
    bool parseModbusResponse(ModbusMessage & modbusMsg, const char * receivedMsg,
            const int & msgLen);
    bool receiveModbusAsciiMessage(std::string& receiveBuffer,
            std::chrono::time_point<std::chrono::high_resolution_clock> expTime);
    std::string binaryToModbusAsciiMessage(int serializedMsgLen,
            unsigned char* asciiMessage);
    void appendToCombinedBuffer(unsigned char combinedBuffer[3000],
            CommDataBuffer *periodicDataPtr, uint16_t &combinedBuffIndex);
    unsigned char * createMultipleBuffer(uint16_t & combinedDataLength);

    uint8_t modbusSlaveAddress; // used as modbus slave address in slave mode,
                                // and current slave that was being polled when in Master mode
    std::list <ModbusSlave *> modbusSlavesList; //list of modbus slaves for master mode.
    std::queue<CommandMsg * > slaveCommandQueue;
    std::mutex commandQueueMutex;
    CommandMsg * getQueuedSlaveCommand();
    bool popQueuedSlaveCommand();
    bool sendQueuedCommand();
    void initializeVariables(ModbusModes mode,
            const std::string &radioPort, const int &slaveAddress);
    int processSingleCommBuffer(const unsigned char *dataBuff,
            const uint16_t &dataLen, unsigned char slaveAddress,
            CommDataBuffer *receivedData);

    bool slaveThreadDone;
    bool masterThreadDone;

    BaudRate baudRate;
    std::chrono::milliseconds modbusResponseTimeout;
    std::chrono::milliseconds modbusMasterPollInterval;
    std::chrono::milliseconds modbusTransmissionTimeout;


    std::string radioSerialPort;
    int commandMessageRetryLimit;
    std::map <unsigned char, uint16_t> latestNpwBufferIdMap;
public:
    void transmitMessage();
    RadioCommunicator(EdgeDevice*, ModbusModes mode,
            const rapidjson::Value &communicatorObj);
    void connect() override;
    void subscribe() override;
    void sendMessage(const char * message, const unsigned int length) override;
    bool processIncomingMessage(const char * message, const int &length) override;
    void disconnect() override;
    void sendQueuedMessagesThread() override;
    void startModbusMaster();
    bool addModbusSlave(uint8_t slaveId, std::string slaveName);
    bool sendModbusCommand(uint8_t slaveAddress, CommandRegister regAddress,
            uint16_t value);
    bool enqueueSlaveCommand(CommandMsg * cmd);
    virtual ~RadioCommunicator();

};
char * binaryToHex(const unsigned char * binString, const int & len);
unsigned char * hexToBinary(const char * hexString, int & len);

bool radioCommTest();
#endif /* RADIOCOMMUNICATOR_H_ */
