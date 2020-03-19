#include <EdgeDevice.h>
#include "communicator.h"
#include "DevConfig.h"

communicator::communicator(EdgeDevice * d) {
    sendMessagesThreadPtr = NULL;
    sendMessagesThreadLoopInterval = std::chrono::milliseconds(100);
    edgeDevicePtr = d;
    npwPacketsToBuffer = kDcNpwNumPack.def;
}

int communicator::enqueueMessage(CommDataBuffer * buff){
    LOG(INFO) << "Going to insert in transmission queue with t: "
            << buff->getTimestamp() << ", id: " << buff->getBufferId()
            << "\t Queue size: "  << transmitQueue.size();

    std::lock_guard<std::mutex> guard(transmitQueueMutex);
    transmitQueue[buff->getBufferId()] = buff;
    return 1;
}

void communicator::setNpwPacketsToBuffer(int32_t v) {
    if (v > kDcNpwNumPack.min and v < kDcNpwNumPack.max) {
        npwPacketsToBuffer = v;
    } else {
        LOG(WARNING) << "NPW_EXP_TIME value out of range";
    }
}

bool communicator::removeMessageFromQueue(int32_t messageId) {
    LOG(INFO) << "Message with ID: " << messageId
            << " is delivered, removing from Queue";
    bool ret = false;
    try {
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        auto msgIterator = transmitQueue.find(messageId);
        if (msgIterator != transmitQueue.end()) {
            delete (msgIterator->second);
            transmitQueue.erase(messageId);
            ret = true;
        } else {
            LOG(WARNING)
                    << "Message not found in transmitQueue to be removed, messageId: "
                    << messageId << "queue size: " << transmitQueue.size();
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in removing message from queue: "
                << e.what();
    }
    return ret;
}

CommDataBuffer * communicator::getQueuedMessage() {
    CommDataBuffer * commPtr = nullptr;
    try {
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        if (transmitQueue.size() > 0) {
            commPtr = transmitQueue.begin()->second;
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in retrieving queued message " << e.what();
    }
    return commPtr;
}
