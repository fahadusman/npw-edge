#include "communicator.h"

#include <fstream>

#include "EdgeDevice.h"
#include "DevConfig.h"


communicator::communicator(EdgeDevice *d, bool bufferPersistence) :
        enableBufferPersistence(bufferPersistence) {
    sendMessagesThreadPtr = NULL;
    sendMessagesThreadLoopInterval = std::chrono::milliseconds(100);
    edgeDevicePtr = d;
    packetsToBuffer = d->getRegisterValue(NPW_NUM_PACK);

    transferCount = 0;
    failedTransferCount = 0;
    communicationTime = std::chrono::milliseconds(0);
    queuedBuffersDirPath = "queued_npw_buffers";
    if (enableBufferPersistence) {
        loadStoredBuffers();
    }
}

bool communicator::saveBufferToFile(CommDataBuffer *buff) {
    if (not enableBufferPersistence) {
        LOG_EVERY_N(INFO, 100) << "enableBufferPersistence is false, not storing buffer to disk";
        return false;
    }
    fs::path filePath = queuedBuffersDirPath
            / std::to_string(buff->getExpiryTime());
    std::ofstream commDataFile;
    bool ret = false;
    try {
        commDataFile.open(filePath, std::fstream::binary | std::fstream::trunc);
        if (!commDataFile.is_open()) {
            std::ios_base::iostate rds = commDataFile.rdstate();
            LOG(ERROR) << "Cannot open new NPW buffer file: " << rds;
            //                return false;
        } else if (commDataFile.good()) {
            //                    and (npwBuffFile.rdstate() & std::ifstream::failbit) != 0) {
            int serialBuffLen = 0;
            unsigned char *serialBuff = buff->serialize(serialBuffLen);
            if (serialBuff != nullptr) {
                commDataFile.write((char*) (serialBuff), serialBuffLen);
                if (commDataFile.good()) {
                    LOG(INFO) << "comm buff saved to file ";
                    ret = true;
                } else {
                    LOG(ERROR) << "Failed to save comm buff";
                }
                delete serialBuff;
            } else {
                LOG(ERROR) << "buff->serialize failed";
            }
        } else {
            LOG(WARNING) << "npwBuffFile is not good";
        }
    } catch (const std::exception &e) {
        LOG(ERROR) << " Exception in saveRegisterMapToFile: " << e.what();
        ret = false;
    }
    commDataFile.flush();
    commDataFile.close();
    return ret;
}

bool communicator::enqueueMessage(CommDataBuffer * buff){
    LOG_EVERY_N(INFO, 100) << "Going to insert in transmission queue with t: "
            << buff->getTimestamp() << ", id: " << buff->getBufferId()
            << "\t Queue size: "  << transmitQueue.size();

    bool ret = false;
    try {
        while(transmitQueue.size() >= packetsToBuffer) {
            LOG(INFO) << "Queue size limit reached, removing packet from queue";
            removeMessageFromQueue(transmitQueue.begin()->second->getBufferId());
        }
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        transmitQueue[buff->getBufferId()] = buff;
        ret = true;
    } catch (const std::exception &e) {
        LOG(ERROR) << "Error inserting message in transmit queue. " << e.what();
        ret = false;
    }

    saveBufferToFile(buff);
    return ret;
}

bool communicator::setNumPacketsToBuffer(int32_t v) {
    if (v > kDcNpwNumPack.min and v < kDcNpwNumPack.max) {
        packetsToBuffer = v;
        return true;
    } else {
        LOG(WARNING) << "NPW_EXP_TIME value out of range";
        return false;
    }
}

void communicator::removeBufferFromDisk(uint64_t expTime) {
    if (not enableBufferPersistence) {
        LOG(INFO) << "enableBufferPersistence is disabled, not removing buffer from disk";
        return;
    }

    // Also remove the buffer from disk
    if (fs::remove(queuedBuffersDirPath / std::to_string(expTime))) {
        LOG(INFO) << "Successfully removed file"
                << (queuedBuffersDirPath / std::to_string(expTime));
    } else {
        LOG(WARNING) << "Failed to remove file"
                << (queuedBuffersDirPath / std::to_string(expTime));
    }
}

bool communicator::removeMessageFromQueue(int32_t messageId) {
    LOG_EVERY_N(INFO, 100) << "Removing message from queue, Message ID: " << messageId;
    bool messageRemoved = false;
    uint64_t expTime = 0;
    try {
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        auto msgIterator = transmitQueue.find(messageId);
        if (msgIterator != transmitQueue.end()) {
            expTime = msgIterator->second->getExpiryTime();
            delete (msgIterator->second);
            transmitQueue.erase(messageId);
            messageRemoved = true;
        } else {
            LOG(WARNING)
                    << "Message not found in transmitQueue to be removed, messageId: "
                    << messageId << "queue size: " << transmitQueue.size();
        }

        // Also remove the buffer from disk
        if (enableBufferPersistence)
            removeBufferFromDisk(expTime);

    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in removing message from queue: "
                << e.what();
    }

    return messageRemoved;
}

CommDataBuffer * communicator::getQueuedMessage() {
    CommDataBuffer * commPtr = nullptr;
    uint64_t currentTimeMS = std::chrono::duration_cast<
            std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    try {
        std::lock_guard<std::mutex> guard(transmitQueueMutex);
        while (transmitQueue.size() > 0) {
            commPtr = transmitQueue.begin()->second;
            if (commPtr->getExpiryTime() == 0 //expiryTime == 0 means, no expiry for this buffer
                    or commPtr->getExpiryTime() > currentTimeMS) {
                return commPtr;
            } else {
                LOG(INFO) << "Removing expired message from queue/disk: "
                        << commPtr->getBufferId();
                if (enableBufferPersistence)
                    removeBufferFromDisk(commPtr->getExpiryTime());
                transmitQueue.erase(commPtr->getBufferId());
                delete commPtr;
                commPtr = nullptr;
            }
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in retrieving queued message " << e.what();
    }
    return commPtr;
}

//Add count and cumulative time for successful communication
void communicator::addCommunicationTime(
        std::chrono::duration<long int,std::ratio<1, 1000000000>> t) {
    communicationTime += t;
    transferCount++;
}

void communicator::incFailedTransferCount() {
    failedTransferCount++;
}

void communicator::getCommunicationStats(int &failureCount, int &successCount,
            uint64_t &totalDurationMs) {
    failureCount = failedTransferCount;
    successCount = transferCount;
    totalDurationMs = std::chrono::duration_cast<std::chrono::milliseconds>
                            (communicationTime).count();
    failedTransferCount = 0;
    transferCount = 0;
    communicationTime = std::chrono::milliseconds(0);
}

bool communicator::loadStoredBuffers(){
    if (not enableBufferPersistence) {
        LOG(WARNING) << "enableBufferPersistence is disabled, "
                << "not trying to load stored buffers from disk";
        return false;
    }

    LOG(INFO) << "Going to load buffers from directory: " << queuedBuffersDirPath;
    try {
        if (fs::exists(queuedBuffersDirPath)
                && fs::is_directory(queuedBuffersDirPath)) {
            fs::directory_iterator bufferDirIterator(queuedBuffersDirPath);
            uint64_t bufferExpTime = 0;
            uint64_t currentTimeMS =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();

            for(auto dataBufferFile:bufferDirIterator) {
                LOG(INFO) << "got: " << dataBufferFile.path().filename();
                try {
                    bufferExpTime = std::stoull(dataBufferFile.path().filename());
                    if (bufferExpTime < currentTimeMS) {
                        LOG(INFO) << "buffer in file: " << dataBufferFile.path() << "is expired, discarding it";
                        if (not fs::remove(dataBufferFile)) {
                            LOG(ERROR) << "Error removing file";
                        }
                        continue;
                    }

                    std::streampos begin, end;
                    std::ifstream dataBuffFile(dataBufferFile.path().c_str(), std::ios::binary);
                    if (dataBuffFile.is_open()) {
                        begin = dataBuffFile.tellg();
                        dataBuffFile.seekg(0, dataBuffFile.end);
                        end = dataBuffFile.tellg();
                        if (end - begin < 100) {
                            LOG(WARNING) << "File size of saved buffer file is too small: "
                                    << end - begin;
                            dataBuffFile.close();
                            if (not fs::remove(dataBufferFile)) {
                                LOG(ERROR) << "Error removing file: " << dataBufferFile;
                            }

                        } else {
                            dataBuffFile.seekg(0, dataBuffFile.beg);
                            char * readBuffer = new char[end-begin];

                            dataBuffFile.read(readBuffer, end-begin);

                            if (dataBuffFile) {
                                LOG(INFO) << "Successfully read data Buffer from file.";
                            } else {
                                LOG(INFO)
                                        << "Unable to read complete data, bytes count: "
                                        << dataBuffFile.gcount();
                                delete readBuffer;
                                readBuffer = nullptr;
                                dataBuffFile.close();
                                continue;
                            }

                            if (readBuffer[0] != (char) buffTypeNpwBuffer) {
                                LOG(ERROR) << "invalid buffer type, discarding file";
                                delete readBuffer;
                                readBuffer = nullptr;
                                dataBuffFile.close();
                                if (not fs::remove(dataBufferFile)) {
                                    LOG(ERROR) << "Error removing file";
                                }
                                continue;
                            }

                            NpwBuffer * npwBuffPtr = new NpwBuffer();
                            if (not npwBuffPtr->deserialize((unsigned char *) readBuffer, end-begin)) {
                                LOG(ERROR) << "cannot deserialize NPW Buffer";
                                delete readBuffer;
                                readBuffer = nullptr;
                                dataBuffFile.close();
                                if (not fs::remove(dataBufferFile)) {
                                    LOG(ERROR) << "Error removing file";
                                }
                                delete npwBuffPtr;
                                npwBuffPtr = nullptr;
                                continue;
                            }

                            enqueueMessage(npwBuffPtr); //TODO: Error handling
                        }
                    } else {
                        LOG(WARNING) << "Unable to open stored NPW buffer file.";
                    }
                    dataBuffFile.close();

                } catch (const std::invalid_argument &e) {
                    LOG(ERROR) << "stoi, invalid argument: "
                            << dataBufferFile.path().filename() << "\tException: "
                            << e.what();
                } catch (const std::out_of_range &e) {
                    LOG(ERROR) << "stoi, out of range: "
                            << dataBufferFile.path().filename() << "\tException: "
                            << e.what();
                }
            }
        } else {
            LOG(INFO) << "Queued NPW Buffers directory not found, creating: "
                    << queuedBuffersDirPath;
            if (fs::create_directories(queuedBuffersDirPath)) {
                LOG(INFO) << "Directory created successfully";
            } else {
                LOG(ERROR) << "Directory not created";
            }
        }
    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in removing message from queue: "
                << e.what();
    }
    return false;
}
