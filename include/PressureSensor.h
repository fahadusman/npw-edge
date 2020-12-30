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
#include "rapidjson/document.h"

#include "Sensor.h"
#include "SensorReading.h"
#include "NpwBuffer.h"
#include "DevConfig.h"
#include "SerialComPort.h"

enum NpwState {noDropDetected, firstDropDetected, secondDropDetected};

class PressureSensor: public Sensor {
private:
	size_t npwBufferLength; //Number of samples in NPW buffer
	std::vector<SensorReading<readingType> *> sensorReadingCircularBuffer;
	unsigned int readingIntervalMs;
	bool recodringValues;
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
    float npwScalingFactor;     //for -1 to 100 PSI pressure range, and uint16
                                // it should be 648.
    float npwScalingOffset;     //This is the initial offset applied to the value
                                // obtained from PT to compensate for negative values
                                // as they have to be transmitted as unsigned integers.
    bool suppressNPWBuffer;
    CommandRegister npwThrCmdRegNo;
    bool breachOnPressureDropOnly;
    // 1 =  only pressure drop events would trigger NPW buffer creation
    // 0 =  pressure difference (rise or fall) will be considered as breach
    //      event when it is greater than currently configured threshold value.

	double readSensorValueDummy();

	void fillCircularBufferWithDummyValues();
    void createNPWBuffer();
    void processIncomingCommand();
    void clearNPWBufferAndState();
    void updateReadingInterval(const int newInterval);
    int applyCommand(CommandMsg * cmd, int oldValue, const DevConfig & dc,
            bool resetNpwThread);
    void updateBufferLengths();
    static int sensorCount;
    unsigned int samplesCountPeriodicAverage;
    bool wasThresholdExceeded;
    std::vector<double> simulatedValues;

public:
	void npwThread();
	void startNpwThread();
	void stopNpwThread();
    PressureSensor(communicator *cPtr, EdgeDevice *ePtr,
            const rapidjson::Value & pressureSensorObj);
    PeriodicValue * getCurrentValue();
	virtual ~PressureSensor();
};


#endif /* INCLUDE_PRESSURESENSOR_H_ */
