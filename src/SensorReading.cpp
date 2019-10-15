/*
 * pressureReading.cpp
 *
 *  Created on: Oct 11, 2019
 *      Author: Fahad Usman
 */


#include "SensorReading.h"

//template <class valueType>
//SensorReading<valueType>::SensorReading(){
//	value = 0;
//	timestampMS = 0;
//}
//
//template <class valueType>
//SensorReading<valueType>::SensorReading(valueType v, __uint32_t t){
//	value = v;
//	timestampMS = t;
//}
//
//template <class valueType>
//SensorReading<valueType>::SensorReading(valueType v){
//	value = v;
//	timestampMS = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
//	LOG(INFO) << "new sensor value: " << value << timestampMS;
//}
//void SensorReading<float>::print(){
//	std::cout << "v: " << value << "t: " << timestampMS;
//}
