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
    int slaveAddress;
    int functionCode;
    int startAddress;
    int quantityOfRegisters;
    int byteCount;
    int * data;
    int registerAddress;
    int writeData;
    int errorCheck;
};

const unsigned int kModbusResponseHdrLength = 11;

class RadioCommunicator: public communicator {
protected:
    SerialStream sStream;
    std::string readBuffer;
    ModbusModes modbusMode;
    void modbusSlaveThread();
    void modbusMasterThread();
    std::thread * slaveThreadPtr;
    bool isConnected;
    bool parseModbusMessage(ModbusMessage &);

public:
    void transmitMessage(CommDataBuffer* commPtr);
    RadioCommunicator(EdgeDevice *);
    void connect() override;
    void subscribe() override;
    void sendMessage(const char * message, const unsigned int length) override;
    bool processIncomingMessage(const char * message, const int length) override;
    void disconnect() override;
    void sendQueuedMessagesThread() override;
    virtual ~RadioCommunicator();

};
char * binaryToHex(const unsigned char * binString, const int & len);
unsigned char * hexToBinary(const char * hexString, int & len);

#endif /* RADIOCOMMUNICATOR_H_ */
