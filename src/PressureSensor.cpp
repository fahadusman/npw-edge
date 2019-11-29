/*
 * pressureSensor.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: Fahad Usman
 */


#include "PressureSensor.h"
#include "simulatedValues.h"
#include "PeriodicValue.h"

#include <iostream>
#include <new>
#include <math.h>

//this function has to be called after adding a new value to the circular buffer
//and before removing the older value.
void PressureSensor::updateMovingAverages() {
	DLOG_EVERY_N(INFO, 50) << "before calculateMovingAverage, firstAverage: " << firstAverage << ", secondAverage: " << secondAverage;
	firstAverage = (firstAverage*(firstAverageEnd-firstAverageStart+1)
			+ sensorReadingCircularBuffer[sensorReadingCircularBuffer.size() -1 - firstAverageStart]->value
			- sensorReadingCircularBuffer[sensorReadingCircularBuffer.size() -1 - firstAverageEnd]->value)
			/ (firstAverageEnd-firstAverageStart+1);

    secondAverage = (secondAverage*(secondAverageEnd-secondAverageStart+1)
            + sensorReadingCircularBuffer[sensorReadingCircularBuffer.size() -1 - secondAverageStart]->value
            - sensorReadingCircularBuffer[sensorReadingCircularBuffer.size() -1 - secondAverageEnd]->value)
            / (secondAverageEnd-secondAverageStart+1);
	DLOG_EVERY_N(INFO, 50) << "After calculateMovingAverage, firstAverage: " << firstAverage << ", secondAverage: " << secondAverage;
}

NpwBuffer* PressureSensor::createNpwBuffer(){
	size_t circularBufferLength = sensorReadingCircularBuffer.size();
//	for (auto i:sensorReadingCircularBuffer) {i->print();}

	LOG(INFO) << "createNewNpwBuffer, circularBufferLength:" << circularBufferLength << "\tNPW buffer len:" << npwBufferLength;

	if (circularBufferLength < npwBufferLength){
		LOG(ERROR) << "Not enough values (" << circularBufferLength << ") in circular buffer to create NPW buffer of " << npwBufferLength;
		return NULL;
	}

	unsigned int startIndex = circularBufferLength - npwBufferLength;
	NpwBuffer * newNpwBufferPtr = new (std::nothrow)
			NpwBuffer(sensorReadingCircularBuffer[startIndex]->timestampMS);
	sensorReadingCircularBuffer[startIndex]->print();

	if (!newNpwBufferPtr){
		LOG(FATAL) << "Unable to allocate memory npw buffer";
		return newNpwBufferPtr;
	}

	for (int i = 0; startIndex + i < circularBufferLength; i++){
//		newNpwBuffer[i] = sensorReadingCircularBuffer[startIndex+i]->value;
		newNpwBufferPtr->insertAt(i, sensorReadingCircularBuffer[startIndex+i]->value);

	}

	return newNpwBufferPtr;
}

double PressureSensor::readSensorValueDummy(){
//	static readingType v =  0.025;
//	std::this_thread::sleep_for(std::chrono::milliseconds(4));
//	return v+=0.0000001; //TODO: This is for testing only, remove this later

	static int i = 0;
	static unsigned int totalValues = sizeof (simulatedValues)/sizeof(int);
	return (double(simulatedValues[i++ % totalValues]))/100000;
}

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

	return result;
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
	// stop npwThread
	return;
}

//PressureSensor::PressureSensor() {
//	npwThreadPtr = NULL;
//	circularBufferLength = kDefaultCircularBufferLength;
//	readingIntervalMs = kDefReadingIntervalMs;
//	recodringValues = false;
//	npwBufferLength = kDefNpwBufferLength;
//	initializeSensor();
//}

PressureSensor::PressureSensor(std::string portName, communicator * cPtr) :
        sPort(portName, kDefaultBaudRate, kDefaultParity, kDefaultBlocking),
        commPtr(cPtr) {
	if (portName == ""){
		LOG(WARNING) << "portName is null, using default port name";
		portName = kDefaultPortName;
	}

	if (not commPtr){
	    LOG(FATAL) << "commPtr is null.";
	}
//	PressureSensor();
	npwThreadPtr = NULL;
	circularBufferLength = kDefaultCircularBufferLength;
	readingIntervalMs = kDefReadingIntervalMs;
	recodringValues = false;
	npwBufferLength = kDefNpwBufferLength;
	initializeSensor();

	firstAverage = 0.0;
	secondAverage = 0.0;

	//The start and end of averages is index from the most recent value in the circular buffer
	firstAverageStart 	= 0;	//first average starts at the most recent value.
	firstAverageEnd 	= 150; 	//3s x 50 samples/s
	secondAverageStart 	= 250; 	//second average starts at t-5
	secondAverageEnd 	= 1250; // second average ends at t-25

	npwDetectionthreshold = kDefNpwThreshold; //threshold
	currentNpwState = noDropDetected;
	totalNPWsDetected = 0;

	t1Ms = kDefT1Ms;
	t2Ms = kDefT2Ms;

	fillCircularBufferWithDummyValues();

	npwBufferCreationTime = std::chrono::high_resolution_clock::now();

}

/*
 * Creates a new NPW buffer after a pressure drop is detected and the required
 * time is passed after the initial detection time.
 */
void PressureSensor::createNPWBuffer(
        const std::chrono::time_point<std::chrono::high_resolution_clock>& currentTimePoint) {
    if (currentNpwState != noDropDetected
            && currentTimePoint > npwBufferCreationTime) {
        LOG(INFO) << "Pressure Drop Detected";
        NpwBuffer* npwBufferPtr = createNpwBuffer();
        commPtr->enqueueMessage(npwBufferPtr);
        LOG(INFO) << "new NPW Buffer created at: "
                << npwBufferPtr->getTimestamp();
        if (currentNpwState == firstDropDetected) {
            currentNpwState = noDropDetected;
        } else if (currentNpwState == secondDropDetected) {
            currentNpwState = firstDropDetected;
            npwBufferCreationTime = currentTimePoint
                    + std::chrono::milliseconds(t1Ms + t2Ms);
        }
    }
}

void PressureSensor::updateNPWState(std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint){
	static bool wasThresholdExceeded = false;
	bool isThresholdExceeded = fabs(firstAverage - secondAverage) > npwDetectionthreshold;
//	LOG_IF(INFO, isThresholdExceeded) << "Threshold exceeded";
	LOG_EVERY_N(INFO, 50) << "wasThresholdExceeded: " << wasThresholdExceeded <<
			"\tisThresholdExceeded: " << isThresholdExceeded << "\tDeltaP: " << firstAverage - secondAverage;
	if ((not wasThresholdExceeded) and isThresholdExceeded){
		DLOG(INFO) << "pressure drop detected, DeltaP: " << firstAverage - secondAverage << "\tNPW State: " << currentNpwState;
		switch (currentNpwState) {
		case noDropDetected:
			currentNpwState = firstDropDetected;
			npwBufferCreationTime = currentTimePoint + std::chrono::milliseconds(t2Ms);
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

uint64_t PressureSensor::sendPeriodicValue(uint64_t currentTime,
        uint64_t previousPeriodicValueTransmitTime,
        double previousPeriodicVal, double currentValue) {
    if (((currentTime > previousPeriodicValueTransmitTime + periodicValMinInterval)
            && (fabs(previousPeriodicVal - currentValue) > periodicValChangeThreshold))
            || currentTime > previousPeriodicValueTransmitTime + periodicValMaxInterval) {

        LOG_EVERY_N(INFO, 10) << "sending periodic value: " << currentValue;
        CommDataBuffer* pValBuffPtr = new PeriodicValue(currentValue,
                currentTime, id);
        commPtr->enqueueMessage(pValBuffPtr);
        previousPeriodicValueTransmitTime = currentTime;
    }
    return previousPeriodicValueTransmitTime;
}

void PressureSensor::npwThread(){
//	std::chrono::time_point<std::chrono::high_resolution_clock> readingTimepoint;
	DLOG(INFO) << "starting npw thread\n";
	double currentValue = 0;
	__uint64_t previousPeriodicValueTransmitTime = 0;
	double previousPeriodicVal = 0;
	SensorReading<double> * sensorReadingPtr = NULL;
	__uint64_t currentTime = 0;
	while(recodringValues){
		std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
				std::chrono::high_resolution_clock::now();

		createNPWBuffer(currentTimePoint);

//		currentValue = readSensorValue();
		currentValue = readSensorValueDummy();
		LOG_EVERY_N(INFO, 50) << "currentValue: " << currentValue;
		currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint.time_since_epoch()).count();
		sensorReadingPtr = new SensorReading<double> (currentValue, currentTime);
		sensorReadingCircularBuffer.push_back(sensorReadingPtr);

        previousPeriodicValueTransmitTime = sendPeriodicValue(currentTime,
                previousPeriodicValueTransmitTime, previousPeriodicVal,
                currentValue);

		updateMovingAverages();
		updateNPWState(currentTimePoint);

		if(sensorReadingCircularBuffer.size() > circularBufferLength){
			LOG_FIRST_N(INFO, 1) << "Circular Buffer full.";
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
		std::this_thread::sleep_until(wakeupTime);
	}

	DLOG(INFO) << "npwThread done";
}

void PressureSensor::startNpwThread(){
	DLOG(INFO) << "PressureSensor::startNpwThread()";
	recodringValues = true;
	npwThreadPtr = new std::thread(&PressureSensor::npwThread, this);
	LOG(INFO) << "New NPW thread launched.";
}

void PressureSensor::stopNpwThread(){
	LOG(INFO) << "PressureSensor::stopNpwThread()";
	recodringValues = false;
	npwThreadPtr->join();

	delete npwThreadPtr;
	npwThreadPtr = NULL;
}

void PressureSensor::fillCircularBufferWithDummyValues(){
	LOG(WARNING) << "initializing circular buffer with dummy values";

	readingType v = 61.482;
	SensorReading<double> * sensorReadingPtr = NULL;
	std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint =
			std::chrono::high_resolution_clock::now() - std::chrono::seconds(25);
	__uint64_t currentTimeMs =
			std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint.time_since_epoch()).count();
	currentTimeMs -= currentTimeMs%10;

	for(unsigned int i = 0; i < circularBufferLength; i++){
		sensorReadingPtr = new (std::nothrow) SensorReading<double> (v, currentTimeMs+=kDefReadingIntervalMs);
		sensorReadingCircularBuffer.push_back(sensorReadingPtr);
	}

	unsigned int i = 0;
//	for (auto x:sensorReadingCircularBuffer){
//		std::cout << i++ << ": ";
//		x->print();
//	}

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
