/*
 * DelayFlag.h
 *
 *  Created on: Mar 18, 2021
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_DELAYFLAG_H_
#define INCLUDE_DELAYFLAG_H_

#include <chrono>
#include <string>

class DelayFlag {
protected:
    std::chrono::time_point<std::chrono::system_clock> triggerTime;
    std::string sensorId;

public:
    std::string isTriggered();
    DelayFlag(const int & delay, const std::string &s);
};

#endif /* INCLUDE_DELAYFLAG_H_ */
