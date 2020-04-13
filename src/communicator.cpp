#include "communicator.h"

#include <fstream>

#include "EdgeDevice.h"
#include "DevConfig.h"


communicator::communicator(EdgeDevice *d, bool bufferPersistence) :
        enableBufferPersistence(bufferPersistence) {
    sendMessagesThreadPtr = NULL;
    sendMessagesThreadLoopInterval = std::chrono::milliseconds(100);
    edgeDevicePtr = d;
    npwPacketsToBuffer = kDcNpwNumPack.def;

    transferCount = 0;
    failedTransferCount = 0;
    communicationTime = std::chrono::milliseconds(0);
    queuedNpwBuffersDirPath = "queued_npw_buffers";
    if (enableBufferPersistence) {
        loadStoredNpwBuffers();
    }
}

bool communicator::saveBufferToFile(CommDataBuffer *buff) {
    if (not enableBufferPersistence) {
        LOG(INFO) << "enableBufferPersistence is false, not storing buffer to disk";
        return false;
    }
    fs::path filePath = queuedNpwBuffersDirPath
            / std::to_string(buff->getExpiryTime());
    std::ofstream npwBuffFile;
    bool ret = false;
    try {
        npwBuffFile.open(filePath, std::fstream::binary | std::fstream::trunc);
        if (!npwBuffFile.is_open()) {
            std::ios_base::iostate rds = npwBuffFile.rdstate();
            LOG(ERROR) << "Cannot open new NPW buffer file: " << rds;
            //                return false;
        } else if (npwBuffFile.good()) {
            //                    and (npwBuffFile.rdstate() & std::ifstream::failbit) != 0) {
            int serialBuffLen = 0;
            unsigned char *serialBuff = buff->serialize(serialBuffLen);
            if (serialBuff != nullptr) {
                npwBuffFile.write((char*) (serialBuff), serialBuffLen);
                if (npwBuffFile.good()) {
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
    npwBuffFile.flush();
    npwBuffFile.close();
    return ret;
}

int communicator::enqueueMessage(CommDataBuffer * buff){
    LOG(INFO) << "Going to insert in transmission queue with t: "
            << buff->getTimestamp() << ", id: " << buff->getBufferId()
            << "\t Queue size: "  << transmitQueue.size();

    //TODO: Remove the oldest message from Queue if queue size limit is reached
    std::lock_guard<std::mutex> guard(transmitQueueMutex);
    transmitQueue[buff->getBufferId()] = buff;

    saveBufferToFile(buff);
    return 1;
}

void communicator::setNpwPacketsToBuffer(int32_t v) {
    if (v > kDcNpwNumPack.min and v < kDcNpwNumPack.max) {
        npwPacketsToBuffer = v;
    } else {
        LOG(WARNING) << "NPW_EXP_TIME value out of range";
    }
}

void communicator::removeBufferFromDisk(uint64_t expTime) {
    if (not enableBufferPersistence) {
        LOG(INFO) << "enableBufferPersistence is disabled, not removing buffer from disk";
        return;
    }

    // Also remove the buffer from disk
    if (fs::remove(queuedNpwBuffersDirPath / std::to_string(expTime))) {
        LOG(INFO) << "Successfully removed file"
                << (queuedNpwBuffersDirPath / std::to_string(expTime));
    } else {
        LOG(WARNING) << "Failed to remove file"
                << (queuedNpwBuffersDirPath / std::to_string(expTime));
    }
}

bool communicator::removeMessageFromQueue(int32_t messageId) {
    LOG(INFO) << "Message with ID: " << messageId
            << " is delivered, removing from Queue";
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
        removeBufferFromDisk(expTime);

    } catch (const std::exception & e) {
        LOG(ERROR) << "Exception in removing message from queue: "
                << e.what();
    }

    return messageRemoved;
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

bool communicator::loadStoredNpwBuffers(){
    if (not enableBufferPersistence) {
        LOG(WARNING) << "enableBufferPersistence is disabled, "
                << "not trying to load stored buffers from disk";
        return false;
    }

    LOG(INFO) << "Going to load NPW buffers from directory: " << queuedNpwBuffersDirPath;
    try {
        if (fs::exists(queuedNpwBuffersDirPath)
                && fs::is_directory(queuedNpwBuffersDirPath)) {
            fs::directory_iterator npwDirIterator(queuedNpwBuffersDirPath);
            uint64_t bufferExpTime = 0;
            uint64_t currentTimeMS =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count();

            for(auto npwFile:npwDirIterator) {
                LOG(INFO) << "got: " << npwFile.path().filename();
                try {
                    bufferExpTime = std::stoul(npwFile.path().filename());
                    if (bufferExpTime < currentTimeMS) {
                        LOG(INFO) << "buffer in file: " << npwFile.path() << "is expired, discarding it";
                        if (not fs::remove(npwFile)) {
                            LOG(ERROR) << "Error removing file";
                        }
                        continue;
                    }

                    std::streampos begin, end;
                    std::ifstream npwBuffFile(npwFile.path().c_str(), std::ios::binary);
                    if (npwBuffFile.is_open()) {
                        begin = npwBuffFile.tellg();
                        npwBuffFile.seekg(0, npwBuffFile.end);
                        end = npwBuffFile.tellg();
                        if (end - begin < 100) {
                            LOG(WARNING) << "File size of npw buffer file is too small: "
                                    << end - begin;
                            npwBuffFile.close();
                            if (not fs::remove(npwFile)) {
                                LOG(ERROR) << "Error removing file: " << npwFile;
                            }

                        } else {
                            npwBuffFile.seekg(0, npwBuffFile.beg);
                            char * readBuffer = new char[end-begin];

                            npwBuffFile.read(readBuffer, end-begin);

                            if (npwBuffFile) {
                                LOG(INFO) << "Successfully read NPW Buffer from file.";
                            } else {
                                LOG(INFO)
                                        << "Unable to read complete data, bytes count: "
                                        << npwBuffFile.gcount();
                                delete readBuffer;
                                readBuffer = nullptr;
                                npwBuffFile.close();
                                continue;
                            }

                            if (readBuffer[0] != (char) buffTypeNpwBuffer) {
                                LOG(ERROR) << "invalid buffer type, discarding file";
                                delete readBuffer;
                                readBuffer = nullptr;
                                npwBuffFile.close();
                                if (not fs::remove(npwFile)) {
                                    LOG(ERROR) << "Error removing file";
                                }
                                continue;
                            }

                            NpwBuffer * npwBuffPtr = new NpwBuffer();
                            if (not npwBuffPtr->deserialize((unsigned char *) readBuffer, end-begin)) {
                                LOG(ERROR) << "cannot deserialize NPW Buffer";
                                delete readBuffer;
                                readBuffer = nullptr;
                                npwBuffFile.close();
                                if (not fs::remove(npwFile)) {
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
                    npwBuffFile.close();

                } catch (const std::invalid_argument &e) {
                    LOG(ERROR) << "stoi, invalid argument: "
                            << npwFile.path().filename() << "\tException: "
                            << e.what();
                } catch (const std::out_of_range &e) {
                    LOG(ERROR) << "stoi, out of range: "
                            << npwFile.path().filename() << "\tException: "
                            << e.what();
                }
            }
        } else {
            LOG(INFO) << "Queued NPW Buffers directory not found, creating: "
                    << queuedNpwBuffersDirPath;
            if (fs::create_directories(queuedNpwBuffersDirPath)) {
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
