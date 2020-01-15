#include <EdgeDevice.h>
#include <iostream>
#include <glog/logging.h>

#include "PressureSensor.h"
#include "MqttHandler.h"
#include "PeriodicValue.h"


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    EdgeDevice edgeDevice;
    edgeDevice.runForever();

    LOG(INFO) << "Terminating NPW app";
	return 0;
}
