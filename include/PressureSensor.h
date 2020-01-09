#ifndef INCLUDE_PRESSURESENSOR_H_
#define INCLUDE_PRESSURESENSOR_H_
/*
 * pressureSensor.h
 *
 *  Created on: Sep 26, 2019
 *      Author: Fahad Usman
 */

#include <string>
#include <vector>
#include <thread>

#include "Sensor.h"
#include "serialPort.h"
#include "SensorReading.h"
#include "NpwBuffer.h"

//const char * kDefaultPTPortName = "/dev/ttyM0";
const unsigned char kKellerInitCommand[] = {1, 48, 52, 0};
const unsigned char kKellerStandardReadCommand[] = {1, 3, 0, 2, 0, 2, 101, 203};
const unsigned char kKellerPropReadCommand[] = {1, 73, 1, 80, 214};

const unsigned int kDefReadingIntervalMs = 20; //must be a factor of 1000 (ms)

const double KPTOffset = +1; // This is the initial offset applied to the value
                             // obtained from PT to compensate for negative values
                             // as they have to be transmitted as unsigned integers.
const double KPTScalingFactor = 1000; // for -1 to 100 PSI pressure range, and uint16
                             // it should be 648.


enum NpwState {noDropDetected, firstDropDetected, secondDropDetected};

class PressureSensor: public Sensor {
private:
	SerialPort sPort;
	size_t npwBufferLength;
	std::vector<SensorReading<readingType> *> sensorReadingCircularBuffer;
	unsigned int readingIntervalMs;
	bool recodringValues;
	double readSensorValue();
	void initializeSensor();
	std::thread * npwThreadPtr;
	NpwBuffer * createNpwBuffer();
	void updateMovingAverages();
	void updateNPWState();

	unsigned int firstAverageStart;
	unsigned int firstAverageEnd;
	unsigned int secondAverageStart;
	unsigned int secondAverageEnd;

	const unsigned int & circularBufferLength = secondAverageEnd;

	readingType firstAverage;
	readingType secondAverage;
	readingType npwDetectionthreshold;

	NpwState currentNpwState;
    unsigned int samplesCountBeforeDetection, samplesCountAfterDetection; //Number of sample to buffer before/after NPW detection
    int remainingSamples;
	unsigned int totalNPWsDetected;
	uint64_t npwBufferExpiryTime;

	uint32_t readSensorValueDummy();
    uint64_t sendPeriodicValue(uint64_t currentTime,
            uint64_t previousPeriodicValueTransmitTime,
            double & previousPeriodicVal, const double & currentValue);

	void fillCircularBufferWithDummyValues();
    void createNPWBuffer();
    void processIncomingCommand();
    void clearNPWBufferAndState();

public:
	void npwThread();
	void startNpwThread();
	void stopNpwThread();
//	PressureSensor();
	PressureSensor(std::string portName, communicator * cPtr);
	virtual ~PressureSensor();
};


#endif /* INCLUDE_PRESSURESENSOR_H_ */
