#include <iostream>
#include <glog/logging.h>
#include <thread>

#include "PressureSensor.h"
#include "SensorReading.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;
    PressureSensor ps("/dev/ttyUSB0");
	ps.startNpwThread();
	std::this_thread::sleep_for(std::chrono::seconds(10));
	ps.stopNpwThread();

	LOG(INFO) << "Terminating NPW app";
	return 0;
}
