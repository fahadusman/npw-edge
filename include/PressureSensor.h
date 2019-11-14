#ifndef INCLUDE_PRESSURESENSOR_H_
#define INCLUDE_PRESSURESENSOR_H_
/*
 * pressureSensor.h
 *
 *  Created on: Sep 26, 2019
 *      Author: Fahad Usman
 */

#include "serialPort.h"
#include <string>
#include "Sensor.h"
#include "SensorReading.h"
#include "NpwBuffer.h"
#include <vector>
#include <thread>
#include <new>

//const char * kDefaultPTPortName = "/dev/ttyM0";
const unsigned char kKellerInitCommand[] = {1, 48, 52, 0};
const unsigned char kKellerStandardReadCommand[] = {1, 3, 0, 2, 0, 2, 101, 203};
const unsigned char kKellerPropReadCommand[] = {1, 73, 1, 80, 214};

const unsigned int kDefReadingIntervalMs = 20; //must be divisible by 1000 (ms)
const unsigned int kDefaultCircularBufferLength = 25 * 1000/kDefReadingIntervalMs;
const double kDefNpwThreshold = 0.02;

enum NpwState {noDropDetected, firstDropDetected, secondDropDetected};

class PressureSensor: public Sensor {
private:
	SerialPort sPort;
	size_t circularBufferLength;
	size_t npwBufferLength;
	std::vector<SensorReading<readingType> *> sensorReadingCircularBuffer;
	unsigned int readingIntervalMs;
	bool recodringValues;
	double readSensorValue();
	void initializeSensor();
	std::thread * npwThreadPtr;
	NpwBuffer * createNpwBuffer();
	void updateMovingAverages();
	void updateNPWState(std::chrono::time_point<std::chrono::high_resolution_clock> currentTimePoint);

	unsigned int firstAverageStart;
	unsigned int firstAverageEnd;
	unsigned int secondAverageStart;
	unsigned int secondAverageEnd;

	readingType firstAverage;
	readingType secondAverage;
	readingType npwDetectionthreshold;

	NpwState currentNpwState;
	std::chrono::time_point<std::chrono::high_resolution_clock> npwBufferCreationTime; //this would be the time pint in future to create npw buffer
	unsigned int t1Ms, t2Ms; // the NPW buffer will contain the pressure values for T1 duration before the event and for T2 duration after the event
	unsigned int totalNPWsDetected;

	double readSensorValueDummy();
	void fillCircularBufferWithDummyValues();
public:
	void npwThread();
	void startNpwThread();
	void stopNpwThread();
//	PressureSensor();
	PressureSensor(std::string portName);
	virtual ~PressureSensor();
};


#endif /* INCLUDE_PRESSURESENSOR_H_ */
