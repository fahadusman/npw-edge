#include <iostream>

#include <glog/logging.h>

#include "EdgeDevice.h"
#include "RadioCommunicator.h"
#include "PeriodicValue.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    Role edgeDeviceRole = gatewayEdgeDevice;
    EdgeDevice edgeDevice(edgeDeviceRole);
    communicator * commPtr = nullptr;
    
    if (edgeDeviceRole == stationEdgeDevice) {
        commPtr = new MqttCommunicator(&edgeDevice);
    } else if (edgeDeviceRole == bvEdgeDevice) {
        commPtr = new RadioCommunicator(&edgeDevice, 1, modbusSlave, "/dev/ttyUSB1");
    }

    if (edgeDeviceRole == gatewayEdgeDevice) {
        commPtr = new MqttCommunicator(&edgeDevice);
        RadioCommunicator * modbusMasterPtr =
                new RadioCommunicator(&edgeDevice, 1, modbusMaster, "/dev/ttyUSB0");
        edgeDevice.setModbusMaster(modbusMasterPtr);
        modbusMasterPtr->connect();
        modbusMasterPtr->startModbusMaster();
    } else {
        PressureSensor * sensorPtr = new PressureSensor("/dev/ttyUSB0", commPtr);
        sensorPtr->startNpwThread();
        edgeDevice.addSensor(sensorPtr);
    }

    commPtr->connect();
    commPtr->subscribe();
    edgeDevice.setCommunicator(commPtr);
    edgeDevice.runForever();

	return 0;
}
