#include <iostream>
#include <glog/logging.h>

#include "PressureSensor.h"
#include "MqttHandler.h"
#include "PeriodicValue.h"

#include "StationEdgeDevice.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    StationEdgeDevice edgeDevice;
    edgeDevice.runForever();

    LOG(INFO) << "Terminating NPW app";
	return 0;
}
