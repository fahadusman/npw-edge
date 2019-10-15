/*
 * pressureReading.h
 *
 *  Created on: Oct 11, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_SENSORREADING_H_
#define INCLUDE_SENSORREADING_H_
#include <cstdint>
#include <chrono>
#include <iostream>
#include <glog/logging.h>

template <class valueType>
class SensorReading {
public:
	valueType value;
	__uint64_t timestampMS;

	void print(){
		std::cout << "v: " << value << "\tt: " << timestampMS << std::endl;
	}

	SensorReading(){
		value = 0;
		timestampMS = 0;
	}

	SensorReading(valueType v, __uint64_t t){
		value = v;
		timestampMS = t;
		LOG_EVERY_N(INFO, 100) << "new sensor value, v: " << value << "\tt:" << timestampMS;
	}
	SensorReading(valueType v){
		value = v;
		auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
		timestampMS = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
		LOG_FIRST_N(INFO, 50) << "new sensor value: " << value << "\t" << timestampMS << "\t" << now.count();
	}

};



#endif /* INCLUDE_SENSORREADING_H_ */
