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

#include "glog/logging.h"

class StationEdgeDevice;

class communicator{
protected:
    std::mutex transmitQueueMutex;
	std::map<uint32_t, CommDataBuffer* > transmitQueue;
    std::thread * sendMessagesThreadPtr;
    std::chrono::duration<int,std::milli> sendMessagesThreadLoopInterval;
    StationEdgeDevice * edgeDevicePtr;

    int32_t npwPacketsToBuffer;

public:
	communicator(StationEdgeDevice * d);
	virtual void sendMessage(const char * message, const unsigned int length) = 0;
	virtual void processIncomingMessage(const char * message, const int length) = 0;
	virtual int enqueueMessage(CommDataBuffer * buff);
	virtual ~communicator(){}
	virtual void connect() = 0;
	virtual void disconnect() = 0;
	virtual void sendQueuedMessagesThread() = 0;
	virtual void subscribe() = 0;
    void setNpwPacketsToBuffer(int32_t v);
    virtual bool removeMessageFromQueue(int32_t messageId);
};



#endif /* INCLUDE_COMMUNICATOR_H_ */
