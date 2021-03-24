/*
 * DelayFlag.cpp
 *
 *  Created on: Mar 18, 2021
 *      Author: Fahad Usman
 */
#include <glog/logging.h>
#include <DelayFlag.h>

DelayFlag::DelayFlag(const int & delay, const std::string & s) {
    triggerTime = std::chrono::system_clock::now()
            + std::chrono::milliseconds(delay);
    sensorId = s;
    LOG(INFO) << "Delay flag constructed: " << s << ", Trigger Time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                    triggerTime.time_since_epoch()).count();
}

//{"PT2003": 1}
std::string DelayFlag::isTriggered() {
    if (triggerTime < std::chrono::system_clock::now()) {
        return "{\"" + sensorId + "_delay\": 1}";
    }
    return "";
}
