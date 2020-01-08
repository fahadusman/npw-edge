#include "communicator.h"
#include "StationEdgeDevice.h"
#include "DevConfig.h"

communicator::communicator(StationEdgeDevice * d) {
    sendMessagesThreadPtr = NULL;
    sendMessagesThreadLoopInterval = std::chrono::milliseconds(100);
    edgeDevicePtr = d;
    npwPacketsToBuffer = kDcNpwNumPack.def;
}

int communicator::enqueueMessage(CommDataBuffer * buff){
    LOG(INFO) << "Going to insert in transmission queue with t: " << buff->getTimestamp() << "\t Queue size: " << transmitQueue.size();
    std::lock_guard<std::mutex> guard(transmitQueueMutex);
    transmitQueue[buff->getBufferId()] = buff;
    return 0;
}

void communicator::setNpwPacketsToBuffer(int32_t v) {
    if (v > kDcNpwNumPack.min and v < kDcNpwNumPack.max) {
        npwPacketsToBuffer = v;
    } else {
        LOG(WARNING) << "NPW_EXP_TIME value out of range";
    }
}

bool communicator::removeMessageFromQueue(int32_t messageId) {
    LOG(INFO) << "Message with ID: " << messageId << " is delivered, and should be removed from Queue";
    return true;
}
