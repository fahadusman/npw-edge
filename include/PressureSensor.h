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
#include "SensorReading.h"
#include "NpwBuffer.h"
#include "DevConfig.h"
#include "SerialComPort.h"

//const char * kDefaultPTPortName = "/dev/ttyM0";
const unsigned char kKellerInitCommand[] = {1, 48, 52, 0};
const unsigned char kKellerStandardReadCommand[] = {1, 3, 0, 2, 0, 2, 101, 203};
const unsigned char kKellerPropReadCommand[] = {1, 73, 1, 80, 214};

const double KPTOffset = +1; // This is the initial offset applied to the value
                             // obtained from PT to compensate for negative values
                             // as they have to be transmitted as unsigned integers.
const double KPTScalingFactor = 1000; // for -1 to 100 PSI pressure range, and uint16
                             // it should be 648.


enum NpwState {noDropDetected, firstDropDetected, secondDropDetected};

class PressureSensor: public Sensor {
private:
	SerialComPort sPort;
	size_t npwBufferLength; //Number of samples in NPW buffer
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

	unsigned int circularBufferLength;

	readingType firstAverage;
	readingType secondAverage;
	readingType npwDetectionthreshold;

	NpwState currentNpwState;
    unsigned int samplesCountBeforeDetection, samplesCountAfterDetection; //Number of sample to buffer before/after NPW detection
    int remainingSamples;
	unsigned int totalNPWsDetected;
	uint64_t npwBufferExpiryTime;   //NPW Buffer Expiry time in ms.

	uint32_t readSensorValueDummy();

	void fillCircularBufferWithDummyValues();
    void createNPWBuffer();
    void processIncomingCommand();
    void clearNPWBufferAndState();
    void updateReadingInterval(const int newInterval);
    int applyCommand(CommandMsg * cmd, int oldValue, const DevConfig & dc,
            bool resetNpwThread);
    void updateBufferLengths();
public:
	void npwThread();
	void startNpwThread();
	void stopNpwThread();
	PressureSensor(std::string portName, communicator * cPtr, EdgeDevice * ePtr);
	virtual ~PressureSensor();
};


#endif /* INCLUDE_PRESSURESENSOR_H_ */
