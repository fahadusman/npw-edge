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

#include "communicator.h"
#include "EdgeDevice.h"

using namespace LibSerial;

enum ModbusModes {
    modbusMaster, modbusSlave
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
};

struct ModbusSlave {
    uint16_t slaveId;
    char modbus0x03Command[18]; //e.g. ":010300000002FA\r\n\0"
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
    bool receiveModbusAsciiMessage(std::string& receiveBuffer);
    std::string binaryToModbusAsciiMessage(int serializedMsgLen,
            unsigned char* asciiMessage);

    uint8_t modbusSlaveAddress; // used as modbus slave address in slave mode,
                                // and current slave that was being polled when in Master mode
    std::list <ModbusSlave *> modbusSlavesList; //list of modbus slaves for master mode.

    bool slaveThreadDone;
    bool masterThreadDone;

//    std::chrono::duration<int, > modbusTimeout;
    std::chrono::milliseconds modbusResponseTimeout;
    std::chrono::milliseconds modbusMasterPollInterval;

    std::string radioSerialPort;
public:
    void transmitMessage();
    RadioCommunicator(EdgeDevice *, const int & slaveAddress, ModbusModes mode, std::string radioPort);
    void connect() override;
    void subscribe() override;
    void sendMessage(const char * message, const unsigned int length) override;
    bool processIncomingMessage(const char * message, const int &length) override;
    void disconnect() override;
    void sendQueuedMessagesThread() override;
    void startModbusMaster();
    bool addModbusSlave(uint8_t slaveId);
    bool sendModbusCommand(uint8_t slaveAddress, CommandRegister regAddress,
            uint16_t value);
    virtual ~RadioCommunicator();

};
char * binaryToHex(const unsigned char * binString, const int & len);
unsigned char * hexToBinary(const char * hexString, int & len);

#endif /* RADIOCOMMUNICATOR_H_ */
