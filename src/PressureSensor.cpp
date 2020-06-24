/*
 * pressureSensor.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: Fahad Usman
 */


#include "PressureSensor.h"

#include <iostream>
#include <new>
#include <math.h>

#include "simulatedValues.h"
#include "PeriodicValue.h"
#include "EdgeDevice.h"

int PressureSensor::sensorCount = 0;

//this function has to be called after adding a new value to the circular buffer
//and before removing the older value.
void PressureSensor::updateMovingAverages() {
//    DLOG_EVERY_N(INFO, 500) << "before calculateMovingAverage, firstAverage: "
//            << firstAverage << ", secondAverage: " << secondAverage;
	try {
        if (sensorReadingCircularBuffer.size() < circularBufferLength) {
            LOG(WARNING)
                    << "Not enough values in circular buffer, cannot calculate averages. Size = "
                    << sensorReadingCircularBuffer.size();
            return;
        }

	    if (firstAverage == -100 and secondAverage == -100) {
	        readingType firstSum = 0, secondSum = 0;
            for (unsigned int i = firstAverageStart; i < firstAverageEnd; i++) {
                firstSum += sensorReadingCircularBuffer[i]->value;
            }
            for (unsigned int i = secondAverageStart; i < secondAverageEnd; i++) {
                secondSum += sensorReadingCircularBuffer[i]->value;
            }

            firstAverage = firstSum/(firstAverageEnd - firstAverageStart);
            secondAverage = secondSum/(secondAverageEnd - secondAverageStart);

            LOG(INFO) << "After initial calculations, firstAverage: " << firstAverage
                    << ", secondAverage: " << secondAverage;
            return;

	    }
        firstAverage = (firstAverage * (firstAverageEnd - firstAverageStart + 1)
                + sensorReadingCircularBuffer[sensorReadingCircularBuffer.size()
                        - 1 - firstAverageStart]->value
                - sensorReadingCircularBuffer[sensorReadingCircularBuffer.size()
                        - 1 - firstAverageEnd]->value)
                / (firstAverageEnd - firstAverageStart + 1);

        secondAverage = (secondAverage
                * (secondAverageEnd - secondAverageStart + 1)
                + sensorReadingCircularBuffer[sensorReadingCircularBuffer.size()
                        - 1 - secondAverageStart]->value
                - sensorReadingCircularBuffer[sensorReadingCircularBuffer.size()
                        - 1 - secondAverageEnd]->value)
                / (secondAverageEnd - secondAverageStart + 1);
    }
    catch (const std::exception & e) {
        LOG(ERROR) << "Exception: " << e.what();
    }
//    DLOG_EVERY_N(INFO, 500)
//    << "After calculateMovingAverage, firstAverage: " << firstAverage
//            << ", secondAverage: " << secondAverage;
}

NpwBuffer* PressureSensor::createNpwBuffer(){
	size_t circularBufferLength = sensorReadingCircularBuffer.size();
	if (circularBufferLength < npwBufferLength){
        LOG(ERROR) << "Not enough values (" << circularBufferLength
                << ") in circular buffer to create NPW buffer of "
                << npwBufferLength;
		return nullptr;
	}

	unsigned int startIndex = circularBufferLength - npwBufferLength;
	NpwBuffer * newNpwBufferPtr = new (std::nothrow)
			NpwBuffer(
            sensorReadingCircularBuffer[startIndex]->timestampMS,
            samplesCountBeforeDetection + samplesCountAfterDetection, id);
	sensorReadingCircularBuffer[startIndex]->print();

	uint64_t currentTime =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

	newNpwBufferPtr->setExpiryTime(npwBufferExpiryTime + currentTime);

	if (!newNpwBufferPtr){
		LOG(FATAL) << "Unable to allocate memory npw buffer";
		return newNpwBufferPtr;
	}

	for (int i = 0; startIndex + i < circularBufferLength; i++){
		newNpwBufferPtr->insertAt(i, sensorReadingCircularBuffer[startIndex+i]->value);
	}

	return newNpwBufferPtr;
}

double PressureSensor::readSensorValueDummy(){
	static int i = 0;
	static unsigned int totalValues = sizeof (simulatedValues)/sizeof(int);
	return (simulatedValues[i++ % totalValues])/100;
}

// Returns a 32-bt integer scaled by KPTScalingFactor

double PressureSensor::readSensorValue(){
    sPort.writeBuffer(kKellerPropReadCommand, sizeof kKellerPropReadCommand);
    unsigned char response[10];
    int bytesRead = sPort.readBuffer(response, sizeof response);

    if(bytesRead != 9){
    	LOG(ERROR) << "Invalid number of bytes read from PT: " << bytesRead;
    	return -0.1;
    }

    float result = 0.0;
    unsigned char * resPtr= reinterpret_cast<unsigned char*>(&result);

    resPtr[0] = response[5];
    resPtr[1] = response[4];
    resPtr[2] = response[3];
    resPtr[3] = response[2];

	return double ((result + KPTOffset) * KPTScalingFactor);
}

void PressureSensor::initializeSensor(){
	LOG(INFO) << "About to write init command.";
    int res = sPort.writeBuffer(kKellerInitCommand, 4);
    DLOG(INFO) << "sPort.writeBuffer returned: " << res;
    unsigned char response[10] = {0};
    int r = sPort.readBuffer(response, sizeof response);
    DLOG(INFO) << "initializeSensor, bytesRead: " << r;
}

PressureSensor::~PressureSensor() {
	// disconnect from sensor and close the socket
    stopNpwThread();
	return;
}

PressureSensor::PressureSensor(communicator *cPtr, EdgeDevice *ePtr,
        const rapidjson::Value &pressureSensorObj) :
        Sensor(cPtr, ePtr, pressureSensorObj["sensor_id"].GetString()), sPort(
                pressureSensorObj["port"].GetString(), B115200,
                kDefaultParity, kDefaultBlocking) {

	if (not commPtr){
	    LOG(FATAL) << "commPtr is null.";
	}

    periodicValMinInterval = edgeDevicePtr->getRegisterValue(MIN_TIME_PERIODIC) * 1000;
    periodicValMaxInterval = edgeDevicePtr->getRegisterValue(MAX_TIME_PERIODIC) * 1000;
    periodicValChangeThreshold = edgeDevicePtr->getRegisterValue(ON_CHANG_THSH_PT);

	npwThreadPtr = nullptr;
	readingIntervalMs = edgeDevicePtr->getRegisterValue(SAMPLE_INTERVAL_NPW);
	recodringValues = false;
    npwBufferLength = edgeDevicePtr->getRegisterValue(NPW_SAMPLE_BEFORE)
            + edgeDevicePtr->getRegisterValue(NPW_SAMPLE_AFTER);
	initializeSensor();

	//The start and end of averages is index from the most recent value in the circular buffer
	firstAverageStart 	= 0;	//first average starts at the most recent value.
	firstAverageEnd 	= edgeDevicePtr->getRegisterValue(NUM_SAMPLES_1_AVG);
	secondAverageStart 	= edgeDevicePtr->getRegisterValue(START_SAMPLE_2_AVG); 	//second average starts at t-5
	secondAverageEnd 	= edgeDevicePtr->getRegisterValue(START_SAMPLE_2_AVG)
	        + edgeDevicePtr->getRegisterValue(NUM_SAMPLES_2_AVG);

    firstAverage = -100.0;
    secondAverage = -100.0;

	npwDetectionthreshold = edgeDevicePtr->getRegisterValue(NPW_THR_PT1); //TODO: Use correct value wrt PT
	currentNpwState = noDropDetected;
	totalNPWsDetected = 0;

	samplesCountBeforeDetection = edgeDevicePtr->getRegisterValue(NPW_SAMPLE_BEFORE);
	samplesCountAfterDetection = edgeDevicePtr->getRegisterValue(NPW_SAMPLE_AFTER);
    remainingSamples = 0;
    updateBufferLengths();

	npwBufferExpiryTime = edgeDevicePtr->getRegisterValue(NPW_EXP_TIME) * 60000; //min to ms

    LOG(INFO) << "Going to add new config to map, Key: " << "NPW_THR_"
            << pressureSensorObj["sensor_id"].GetString()
            << "\tValue: " << (CommandRegister)((int)NPW_THR_PT1 + sensorCount);
    edgeDevicePtr->addConfigToConfigMqp(
        std::string("NPW_THR_")
                + pressureSensorObj["sensor_id"].GetString(),
        edgeDevicePtr->getDeviceId(), (CommandRegister)((int)NPW_THR_PT1 + sensorCount));
    sensorCount++;

	startNpwThread();
}

/*
 * Creates a new NPW buffer after a pressure drop is detected and the required
 * time is passed after the initial detection time.
 */
void PressureSensor::createNPWBuffer() {
    if (currentNpwState != noDropDetected) {
        if (remainingSamples <= 0) {
            NpwBuffer* npwBufferPtr = createNpwBuffer();
            if (npwBufferPtr) {
                commPtr->enqueueMessage(npwBufferPtr);
                LOG(INFO) << "new NPW Buffer created at: "
                        << npwBufferPtr->getTimestamp();
            } else {
                LOG(ERROR) << "create NPW Buffer failed.";
            }
            if (currentNpwState == firstDropDetected) {
                currentNpwState = noDropDetected;
            } else if (currentNpwState == secondDropDetected) {
                currentNpwState = firstDropDetected;
                remainingSamples = samplesCountBeforeDetection
                        + samplesCountAfterDetection;
            }
        } else {
            remainingSamples--;
        }
    }
}

void PressureSensor::updateNPWState(){
	static bool wasThresholdExceeded = false;
	if (circularBufferLength > sensorReadingCircularBuffer.size()) {
//	    LOG_EVERY_N(INFO, 50) << "Not enough values in Circular buffer yet, size: " << sensorReadingCircularBuffer.size();
	    currentNpwState = noDropDetected;
	    return;
	}

	updateMovingAverages();

	bool isThresholdExceeded = fabs(firstAverage - secondAverage) > npwDetectionthreshold;
//	LOG_EVERY_N(INFO, 50) << "wasThresholdExceeded: " << wasThresholdExceeded <<
//			"\tisThresholdExceeded: " << isThresholdExceeded << "\tDeltaP: " << firstAverage - secondAverage;
	if ((not wasThresholdExceeded) and isThresholdExceeded){
		DLOG(INFO) << "pressure drop detected, DeltaP: " << firstAverage - secondAverage << "\tNPW State: " << currentNpwState;
		switch (currentNpwState) {
		case noDropDetected:
			currentNpwState = firstDropDetected;
			remainingSamples = samplesCountAfterDetection;
			break;
		case firstDropDetected:
			currentNpwState = secondDropDetected;
			break;
		case secondDropDetected:
			DLOG(INFO) << "Ignoring pressure drop, 2nd drop already detected.";
			break;
		default:
			LOG(FATAL) << "Invalid currentNpwState: " << currentNpwState;
		}
	}
	wasThresholdExceeded = isThresholdExceeded;
}

void PressureSensor::npwThread(){
	DLOG(INFO) << "starting npw thread\n";
	__uint64_t previousPeriodicValueTransmitTime = 0;
	double previousPeriodicVal = 0;
	SensorReading<double> * sensorReadingPtr = NULL;

	clearNPWBufferAndState();

	while(recodringValues){
		std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
				std::chrono::high_resolution_clock::now();

		createNPWBuffer();

//		currentValue = readSensorValue();
		currentValue = readSensorValueDummy();
		LOG_EVERY_N(INFO, 1000) << "currentValue: " << currentValue;
		currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint.time_since_epoch()).count();
		sensorReadingPtr = new SensorReading<double> (currentValue, currentTime);
		sensorReadingCircularBuffer.push_back(sensorReadingPtr);

		if (enablePeriodicValues) {
            previousPeriodicValueTransmitTime = sendPeriodicValue(currentTime,
                    previousPeriodicValueTransmitTime, previousPeriodicVal,
                    currentValue);
		}

		updateNPWState();

		while (sensorReadingCircularBuffer.size() > circularBufferLength){
			LOG_FIRST_N(INFO, 10) << "Circular Buffer full.";
			sensorReadingPtr = sensorReadingCircularBuffer.front();
			sensorReadingCircularBuffer.erase(sensorReadingCircularBuffer.begin());	//remove the front element
			delete sensorReadingPtr;
			sensorReadingPtr = NULL;
		}

		unsigned long long currentTimeNsCount =
				std::chrono::duration_cast <std::chrono::nanoseconds> (currentTimePoint.time_since_epoch()).count();
		auto wakeupTime = currentTimePoint + std::chrono::milliseconds(readingIntervalMs);
		wakeupTime -= std::chrono::nanoseconds((currentTimeNsCount%10000000)); //round it up to 10ms
		LOG_FIRST_N(INFO, 10) << "wakeupTime: " <<
				std::chrono::duration_cast<std::chrono::nanoseconds>(wakeupTime.time_since_epoch()).count();

		processIncomingCommand();

		std::this_thread::sleep_until(wakeupTime);
	}

	DLOG(INFO) << "npwThread done";
}

void PressureSensor::startNpwThread(){
	DLOG(INFO) << "PressureSensor::startNpwThread()";
	if (npwThreadPtr != NULL) {
	    LOG(INFO) << "Stopping previous NPW Thread before starting new one.";
	    stopNpwThread();
	    npwThreadPtr = NULL;
	}
	recodringValues = true;
	npwThreadPtr = new std::thread(&PressureSensor::npwThread, this);
	LOG(INFO) << "New NPW thread launched.";
}

void PressureSensor::stopNpwThread(){
	LOG(INFO) << "PressureSensor::stopNpwThread()";
	recodringValues = false;
	npwThreadPtr->join();
	clearNPWBufferAndState();
	delete npwThreadPtr;
	npwThreadPtr = NULL;
}

void PressureSensor::fillCircularBufferWithDummyValues(){
	LOG(WARNING) << "initializing circular buffer with dummy values";

    readingType v = 61482;
    SensorReading<double> * sensorReadingPtr = NULL;
    std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
            std::chrono::high_resolution_clock::now()
                    - std::chrono::seconds(25);
    __uint64_t currentTimeMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                    currentTimePoint.time_since_epoch()).count();
    currentTimeMs -= currentTimeMs % 10;

    for (unsigned int i = 0; i < circularBufferLength; i++) {
        sensorReadingPtr = new (std::nothrow) SensorReading<double>(v,
                currentTimeMs += readingIntervalMs);
        sensorReadingCircularBuffer.push_back(sensorReadingPtr);
    }

	unsigned int i = 0;

	unsigned int end = sensorReadingCircularBuffer.size();
	double firstSum = 0.0;
	for (i = end-firstAverageEnd; i < end-firstAverageStart; i++){
		firstSum += sensorReadingCircularBuffer[i]->value;
	}
	firstAverage = firstSum/(firstAverageEnd-firstAverageStart);

	double secondSum = 0.0;
	for (i = end-secondAverageEnd; i < end-secondAverageStart; i++){
		secondSum += sensorReadingCircularBuffer[i]->value;
	}
	secondAverage = secondSum/(secondAverageEnd-secondAverageStart);

	DLOG(INFO) << "firstAverage: " << firstAverage << "\tsecondAverage: " << secondAverage;
}

void PressureSensor::updateReadingInterval(const int newInterval) {
    if ((const unsigned int) newInterval != readingIntervalMs
            and newInterval >= kDcSampleIntervalNpw.min
            and newInterval <= kDcSampleIntervalNpw.max
            and newInterval % 10 == 0) {
        readingIntervalMs = newInterval;
        clearNPWBufferAndState();
    } else {
        LOG(WARNING) << "Discarding newInterval: " << newInterval
                << ", and keeping existing readingIntervalMs: "
                << readingIntervalMs;
    }
}

int PressureSensor::applyCommand(CommandMsg * cmd, int oldValue,
        const DevConfig & dc, bool resetNpwThread) {
    if (oldValue != cmd->getData() and cmd->getData() >= dc.min
            and cmd->getData() <= dc.max) {
        if (resetNpwThread) {
            clearNPWBufferAndState();
        }
        return cmd->getData();
        edgeDevicePtr->updateRegisterValue(cmd);
    }
    LOG(WARNING) << "Not applying newValue: " << cmd->getData()
            << ", and keeping oldValue: " << oldValue;
    return oldValue;
}

void PressureSensor::processIncomingCommand() {
    int secondAverageSampleCount = 0;
    try{
        NpwBuffer* npwBufferPtr = NULL;
        std::lock_guard<std::mutex> guard(commandQueueMutex);
        if (not incomingCommandQueue.empty()){
            CommandMsg * c = incomingCommandQueue.front();
            LOG(INFO) << "Processing command: " << c->getCommand()
                    << "\tData: " << c->getData();

            switch (c->getCommand()) {
            case MAX_TIME_PERIODIC:
                if (c->getData() >= kDcMaxTimePeriodic.min
                        and c->getData() <= kDcMaxTimePeriodic.max) {
                    periodicValMaxInterval = c->getData() * 1000;
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "MAX_TIME_PERIODIC value out of range: "
                            << c->getData();
                break;
            case MIN_TIME_PERIODIC:
                if (c->getData() >= kDcMinTimePeriodic.min
                        and c->getData() <= kDcMinTimePeriodic.max) {
                    this->periodicValMinInterval = c->getData() * 1000;
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "MIN_TIME_PERIODIC value out of range: "
                            << c->getData();
                break;
            case ON_CHANG_THSH_PT:
                periodicValChangeThreshold = applyCommand(c,
                        periodicValChangeThreshold, kDcOnChangThshPt, false);
                break;
            case NPW_EXP_TIME:
                if (c->getData() > kDcNpwExpiryTime.min
                        and c->getData() < kDcNpwExpiryTime.max) {
                    //Expiry time would be sent in minutes, we need to convert it to milliseconds
                    this->npwBufferExpiryTime = c->getData() * 60 * 1000;
                    edgeDevicePtr->updateRegisterValue(c);
                } else
                    LOG(WARNING) << "NPW_EXP_TIME value out of range: "
                            << c->getData();
                break;
            case SAMPLE_INTERVAL_NPW:
                updateReadingInterval(c->getData());
                edgeDevicePtr->updateRegisterValue(c);
                break;
            case NUM_SAMPLES_1_AVG:
                firstAverageEnd = applyCommand(c, firstAverageEnd,
                        kDcNumSamples1stAvg, true);
                break;
            case NUM_SAMPLES_2_AVG:
                secondAverageSampleCount = secondAverageEnd - secondAverageStart;
                secondAverageSampleCount = applyCommand(c, secondAverageSampleCount,
                        kDcNumSamples2ndAvg, true);
                secondAverageEnd = secondAverageStart + secondAverageSampleCount;
                updateBufferLengths();
                break;
            case START_SAMPLE_2_AVG:
                secondAverageSampleCount = secondAverageEnd
                        - secondAverageStart;
                secondAverageStart = applyCommand(c,
                        secondAverageStart, kDcStartSample2ndAvg, true);
                secondAverageEnd = secondAverageStart
                        + secondAverageSampleCount;
                break;
            case NPW_THR_PT1:
            case NPW_THR_PT2:
            case NPW_THR_PT3:
            case NPW_THR_PT4:
                npwDetectionthreshold = applyCommand(c, npwDetectionthreshold,
                        kDcNpwPtThsh, true);
                //TODO: check pt id first
                break;
            case NPW_SAMPLE_AFTER:
                samplesCountAfterDetection = applyCommand(c,
                        samplesCountAfterDetection, kDcNpwSampleAfter, true);
                updateBufferLengths();
                break;
            case NPW_SAMPLE_BEFORE:
                samplesCountBeforeDetection = applyCommand(c,
                        samplesCountBeforeDetection, kDcNpwSampleBefore, true);
                updateBufferLengths();
                break;
            case TEST_FLAG:
                npwBufferPtr = createNpwBuffer();
                if (npwBufferPtr != NULL) {
                    commPtr->enqueueMessage(npwBufferPtr);
                    LOG(INFO) << "Received TEST_FLAG, force creating NPW Buffer";
                }
                break;
            default:
                LOG(WARNING) << "Unhandled command received.";
            }

            incomingCommandQueue.pop();
            delete c;
        }
    }
    catch (const std::exception & e) {
        LOG(ERROR) << "Exception: " << e.what();
    }

}

void PressureSensor::clearNPWBufferAndState() {
    LOG(INFO) << "PressureSensor::clearNPWBufferAndState()";
    try {
        for (auto sensorReading: sensorReadingCircularBuffer) {
            delete sensorReading;
        }
        sensorReadingCircularBuffer.clear();
        currentNpwState = noDropDetected;
        firstAverage = -100;
        secondAverage = -100;
        remainingSamples = 0;
    } catch (const std::exception & e) {
        LOG(ERROR) << "exception: " << e.what();
    }

}

void PressureSensor::updateBufferLengths() {
    npwBufferLength = samplesCountAfterDetection
            + samplesCountBeforeDetection;
    LOG(INFO) << "current circularBufferLength: " << circularBufferLength;
    circularBufferLength =
            (secondAverageEnd > npwBufferLength) ?
                    secondAverageEnd : npwBufferLength;

    LOG(INFO) << "updatedCircularBufferLength: "
            << circularBufferLength;
}
