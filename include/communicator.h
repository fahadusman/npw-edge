/*
 * communicator.h
 *
 *  Created on: Oct 17, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_COMMUNICATOR_H_
#define INCLUDE_COMMUNICATOR_H_

#include "CommDataBuffer.h"

#include <map>
#include <mutex>
#include <thread>
#include <experimental/filesystem>

#include "glog/logging.h"

namespace fs = std::experimental::filesystem;

class EdgeDevice;

class communicator{
protected:
    std::mutex transmitQueueMutex;
	std::map<uint32_t, CommDataBuffer* > transmitQueue;
    std::thread * sendMessagesThreadPtr;
    std::chrono::duration<int,std::milli> sendMessagesThreadLoopInterval;
    EdgeDevice * edgeDevicePtr;

    int32_t npwPacketsToBuffer;

    std::chrono::duration<long int, std::ratio<1, 1000000000>> communicationTime;
    int transferCount;
    int failedTransferCount;

    fs::path queuedNpwBuffersDirPath;
    void addCommunicationTime(
            std::chrono::duration<long int, std::ratio<1, 1000000000>> t);
    void incFailedTransferCount();
    bool loadStoredNpwBuffers();
    bool saveBufferToFile(CommDataBuffer *buff);
    void removeBufferFromDisk(uint64_t expTime);

    bool enableBufferPersistence;

public:
	communicator(EdgeDevice * d, bool bufferPersistence);
	virtual void sendMessage(const char * message, const unsigned int length) = 0;
	virtual bool processIncomingMessage(const char * message, const int & length) = 0;
	virtual int enqueueMessage(CommDataBuffer * buff);
	virtual ~communicator(){}
	virtual void connect() = 0;
	virtual void disconnect() = 0;
	virtual void sendQueuedMessagesThread() = 0;
	virtual void subscribe() = 0;
    void setNpwPacketsToBuffer(int32_t v);
    virtual bool removeMessageFromQueue(int32_t messageId);
    CommDataBuffer * getQueuedMessage();
    void getCommunicationStats(int &failureCount, int &successCount,
            uint64_t &totalDurationMs);
};



#endif /* INCLUDE_COMMUNICATOR_H_ */
