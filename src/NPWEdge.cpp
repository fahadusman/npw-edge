#include <iostream>

#include <glog/logging.h>

#include "EdgeDevice.h"
#include "RadioCommunicator.h"
#include "PeriodicValue.h"
#include "TemperatureSensor.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "starting LDS app, argc = " << argc;

    Role edgeDeviceRole = gatewayEdgeDevice;
    int edgeDeviceId = 1;
    edgeDeviceRole = bvEdgeDevice; edgeDeviceId = 2;

    EdgeDevice edgeDevice(edgeDeviceId, edgeDeviceRole);
    communicator * commPtr = nullptr;

    if (edgeDeviceRole == stationEdgeDevice) {
        commPtr = new MqttCommunicator(&edgeDevice);
    } else if (edgeDeviceRole == bvEdgeDevice) {
        commPtr = new RadioCommunicator(&edgeDevice, 1, modbusSlave, "/dev/pts/2");
    }

    if (edgeDeviceRole == gatewayEdgeDevice) {
        commPtr = new MqttCommunicator(&edgeDevice);
        RadioCommunicator * modbusMasterPtr =
                new RadioCommunicator(&edgeDevice, 1, modbusMaster, "/dev/pts/0");
        modbusMasterPtr->addModbusSlave(1);

        edgeDevice.setModbusMaster(modbusMasterPtr);
        modbusMasterPtr->connect();
        modbusMasterPtr->startModbusMaster();
    } else {
        Sensor *sensorPtr = new PressureSensor("/dev/ttyUSB0", commPtr,
                &edgeDevice, "PT1");
        edgeDevice.addSensor(sensorPtr);
    }

    commPtr->connect();
    commPtr->subscribe();
    edgeDevice.setCommunicator(commPtr);
    edgeDevice.runForever();

	return 0;
}
