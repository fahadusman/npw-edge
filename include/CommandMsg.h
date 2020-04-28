/*
 * CommandMsg.h
 *
 *  Created on: Dec 6, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_COMMANDMSG_H_
#define INCLUDE_COMMANDMSG_H_

#include <cstdint>

#define COMMAND_KEY "command"
#define VALUE_KEY "value"
#define DEVICEID_KEY "device_id" //device_id = 0 means that the command is for all devices

enum CommandRegister {
    UNINITIALIZED_CMD = 0,
    NPW_NUM_PACK = 1, //  In case communications are lost, each edge device will store this many packets.
    NPW_EXP_TIME = 2, //  If any NPW packet stored on edge device is this many minutes old, it'll be discarded.
    MAX_TIME_PERIODIC = 3, //  If the periodic data is not changing, it'll be transmitted after this much time anyway (Seconds).
    MIN_TIME_PERIODIC = 4, //  The minimum time between periodic data is this value.
    ON_CHANG_THSH_PT = 5, //  The periodic pressure value will be sent only if it changes by this much value or if the max time exceeds
    ON_CHANG_THSH_TT = 7, //  The periodic temperature value will be sent only if it changes by this much value or if the max time exceeds
    SAMPLE_INTERVAL_NPW = 9, //  The sampling interval (in ms) of PT values for NPW detection (must be a multiple of 10ms.
    NUM_SAMPLES_1_AVG = 10, //  Number of Samples in 1st average for NPW detection.
    NUM_SAMPLES_2_AVG = 11, //  Number of Samples in 2nd average for NPW detection.
    START_SAMPLE_2_AVG = 12,   //  1st sample for 2nd average.
    NPW_SAMPLE_BEFORE = 13,  //  Number of sample to buffer before NPW detection
    NPW_SAMPLE_AFTER = 14,   //  Number of sample to buffer after NPW detection
    TEST_FLAG = 15, //  When value changes from False to True, a dummy NPW packet will be sent
    REBOOT_TIME = 16, //  For non-negative values, the edge device will be rebooted after this many seconds.
    HEARTBEAT_INTERVAL = 17,   //  Interval for Edge device Heartbeat
    ACK_NPW_BUFF = 18,   // Acknowledgment of successfully receiving an NPW buffer with bufferId so that sender removes it from transmission queue.
    NPW_THR_PT1 = 19,  //  negative pressure drop threshold for detecting NPW for PT1
    NPW_THR_PT2 = 20,  //  negative pressure drop threshold for detecting NPW for PT2
    NPW_THR_PT3 = 21,  //  negative pressure drop threshold for detecting NPW for PT3
    NPW_THR_PT4 = 22,   //  negative pressure drop threshold for detecting NPW for PT4

    EDGE_START_TIME = 31,   //Epoch time of start-up in seconds
    EDGE_RAM = 32,   // Percentage of total RAM
    EDGE_STORAGE = 33,   // Percentage of total persistent storage free
    EDGE_CPU = 34,   // Average CPU Utilization since last HB
    EDGE_CPU_TEMP = 35,   // Current CPU Temperature
    EDGE_GPS_SYNC_STATUS = 36,   // 1 = synched, 0 = not synched
    EDGE_GPS_SYNC_VAL = 37,   // GPS sync accuracy
    EDGE_MW_COMM_STATUS = 38,   //
    BAD_VAL_COUNT = 39, //  sensor bad values according to Tag BAD_VAL_THSH_PT since last HB
    BAD_CRC_COUNT = 40,   //  bad crc values for sensors since last HB
    NO_RSPNS_COUNT = 41,   //  no response count since last HB
    FLAG_NPW_SUPPRESS = 42,   //  Do not sent NPW Buffers if this flag is set
    COMM_FAILURE_COUNT = 43,      //  Failed message count since last HB
    COMM_SUCCESS_COUNT = 44,      //  Successful message count since last HB
    AVERAGE_TRANS_TIME = 45,      //  Average transmission time of one message since last HB
    QUEUED_NPW_BUFFERS = 46,    // Number of NPW Buffers queued for transmission

    INVALID_COMMAND
};

class CommandMsg {
private:
    CommandRegister command; //this is same as modbus register address
    int32_t data;
    uint8_t deviceId;
public:
    CommandMsg();
    CommandMsg(CommandRegister cmd, uint32_t d);
    CommandMsg(CommandRegister cmd, uint32_t d, uint8_t devId);
    CommandRegister getCommand() {
        return command;
    }
    int32_t getData() {
        return data;
    }
    int32_t getSlaveId() {
        return deviceId;
    }
};

#endif /* INCLUDE_COMMANDMSG_H_ */
