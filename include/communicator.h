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
#include "glog/logging.h"
#include <mutex>
#include <thread>

class communicator{
protected:
    std::mutex transmitQueueMutex;
	std::map<uint32_t, CommDataBuffer* > transmitQueue;
    std::thread * sendMessagesThreadPtr;
    std::chrono::duration<int,std::milli> sendMessagesThreadLoopInterval;

public:
	communicator(){
        sendMessagesThreadPtr = NULL;
        sendMessagesThreadLoopInterval = std::chrono::milliseconds(100);
	}
	virtual void sendMessage(const char * message, const unsigned int length) = 0;
	virtual int enqueueMessage(CommDataBuffer * buff){
	    LOG(INFO) << "Going to insert in transmission queue with t: " << buff->getTimestamp() << "\t Queue size: " << transmitQueue.size();
	    std::lock_guard<std::mutex> guard(transmitQueueMutex);
	    transmitQueue[buff->getBufferId()] = buff;
//	    auto ret = transmitQueue.insert(std::pair<uint64_t, CommDataBuffer * >(buff->getTimestamp(), buff));
	    return 0;
	}
	virtual ~communicator(){}
	virtual void connect() = 0;
	virtual void disconnect() = 0;
	virtual void sendQueuedMessagesThread() = 0;
};



#endif /* INCLUDE_COMMUNICATOR_H_ */
