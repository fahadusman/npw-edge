/*
 * CommandMsg.cpp
 *
 *  Created on: Dec 9, 2019
 *      Author: Fahad Usman
 */

#include "CommandMsg.h"

CommandMsg::CommandMsg() {
    command = INVALID_CMD;
    data = 0;
}
CommandMsg::CommandMsg(CommandRegister cmd, uint32_t d){
    if (cmd <= NPW_THR_PT4) {   //need to come up with a better condition.
        command = cmd;
    }
    else {
        command = INVALID_CMD;
    }
    data = d;
}
