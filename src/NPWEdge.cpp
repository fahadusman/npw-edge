#include <iostream>

#include <glog/logging.h>

#include "EdgeDevice.h"
#include "RadioCommunicator.h"
#include "PeriodicValue.h"
#include "TemperatureSensor.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    EdgeDevice e("config_gateway.json");
    e.runForever();

    return 0;
}
