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

class communicator{
protected:
	std::map<uint64_t, CommDataBuffer* > transmitQueue;
public:
	communicator(){}
	virtual void sendMessage(const char * message, const unsigned int length) = 0;
	virtual int enqueueMessage(CommDataBuffer * buff){
	    auto ret = transmitQueue.insert(std::pair<uint64_t, CommDataBuffer * >(buff->getTimestamp(), buff));
	    LOG(INFO) << "insert in transmission queue with t: " << buff->getTimestamp() << "\t Queue size: " << transmitQueue.size();
	    return 0;
	}
	virtual ~communicator(){}
	virtual void connect() = 0;
	virtual void disconnect() = 0;
};



#endif /* INCLUDE_COMMUNICATOR_H_ */
