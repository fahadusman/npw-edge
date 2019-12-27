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

enum CommandRegister {
    UNINITIALIZED_CMD = 0,
    NPW_NUM_PACK = 1, //  In case communications are lost, each edge device will store this many packets.
    NPW_EXP_TIME = 2, //  If any NPW packet stored on edge device is this many minutes old, it'll be discarded.
    MAX_TIME_PERIODIC = 3, //  If the periodic data is not changing, it'll be transmitted after this much time anyway (Seconds).
    MIN_TIME_PERIODIC = 4, //  The minimum time between periodic data is this value.
    ON_CHANG_THSH_PT = 5, //  The periodic pressure value will be sent only if it changes by this much value or if the max time exceeds
    ON_CHANG_THSH_TT = 7, //  The periodic temperature value will be sent only if it changes by this much value or if the max time exceeds
    SAMPL_FREQ_NPW = 9, //  The sampling frequency of PT values for NPW detection.
    NUM_SAMPLES_1_AVG = 10, //  Number of Samples in 1st average for NPW detection.
    NUM_SAMPLES_2_AVG = 11, //  Number of Samples in 2nd average for NPW detection.
    START_SAMPLE_2_AVG = 12,   //  1st sample for 2nd average.
    NPW_SAMPLE_BEFORE = 13,  //  Number of sample to buffer before NPW detection
    NPW_SAMPLE_AFTER = 14,   //  Number of sample to buffer after NPW detection
    TEST_FLAG = 15, //  When value changes from False to True, a dummy NPW packet will be sent
    REBOOT_TIME = 16, //  For non-negative values, the edge device will be rebooted after this many seconds.
    HEARTBEAT_INTERVAL = 17,   //  Interval for Edge device Heartbeat
    ACK_NPW_BUFF = 18,   //  Acknowledgment by server for receiving NPW buffer
    NPW_THR_PT1 = 19,  //  negative pressure drop threshold for detecting NPW for PT1
    NPW_THR_PT2 = 20,  //  negative pressure drop threshold for detecting NPW for PT2
    NPW_THR_PT3 = 21,  //  negative pressure drop threshold for detecting NPW for PT3
    NPW_THR_PT4 = 22,   //  negative pressure drop threshold for detecting NPW for PT4
    INVALID_COMMAND
};

class CommandMsg {
private:
    CommandRegister command; //this is same as modbus register address
    int32_t data;
public:
    CommandMsg();
    CommandMsg(CommandRegister cmd, uint32_t d);
    CommandRegister getCommand() {
        return command;
    }
    int32_t getData() {
        return data;
    }
};

#endif /* INCLUDE_COMMANDMSG_H_ */
