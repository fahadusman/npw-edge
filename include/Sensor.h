#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

/*
 * sensor.h
 *
 *  Created on: Sep 25, 2019
 *      Author: Fahad Usman
 */

class Sensor {
public:
	virtual ~Sensor() = 0;
	virtual double readSensorValue() = 0;
	virtual void initializeSensor() = 0;
//	{
//		return 1.0;
//	}
	Sensor();
protected:
	double currentValue;
};

inline Sensor::~Sensor()
{
	// disconnect from sensor and close the socket
}

#endif /* INCLUDE_SENSOR_H_ */
