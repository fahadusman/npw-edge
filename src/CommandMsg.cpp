/*
 * CommandMsg.cpp
 *
 *  Created on: Dec 9, 2019
 *      Author: Fahad Usman
 */

#include "CommandMsg.h"

#include <glog/logging.h>

CommandMsg::CommandMsg() {
    command = UNINITIALIZED_CMD;
    data = 0;
    deviceId = 0;
    retryCount = 0;
}

CommandMsg::CommandMsg(CommandRegister cmd, uint32_t d):deviceId(0) {
    LOG(INFO) << "New command message, cmd: " << cmd << " data: " << d;
    if (cmd >= UNINITIALIZED_CMD and cmd < INVALID_COMMAND) {
        command = cmd;
    }
    else {
        command = INVALID_COMMAND;
        LOG(WARNING) << "Received invalid command";
    }
    data = d;
    retryCount = 0;
}

CommandMsg::CommandMsg(CommandRegister cmd, uint32_t d, uint8_t devId):CommandMsg(cmd, d) {
    deviceId = devId;
}
