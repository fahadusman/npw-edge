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
//    edgeDeviceRole = stationEdgeDevice; edgeDeviceId = 3;

    EdgeDevice edgeDevice(edgeDeviceId, edgeDeviceRole);
    communicator * commPtr = nullptr;

//    commPtr = new MqttCommunicator(&edgeDevice);
//    commPtr->connect();
//    commPtr->subscribe();
//    edgeDevice.setCommunicator(commPtr);
//    return 0;

//    TemperatureSensor tempSensor("/dev/pts/2", commPtr, &edgeDevice, "TT1");
//    tempSensor.startThread();
//    edgeDevice.runForever();
//
//    return 0;
    
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
//        modbusMasterPtr->addModbusSlave(2);
//        modbusMasterPtr->addModbusSlave(3);

        edgeDevice.setModbusMaster(modbusMasterPtr);
        modbusMasterPtr->connect();
        modbusMasterPtr->startModbusMaster();
    } else {
        Sensor *sensorPtr = new PressureSensor("/dev/ttyUSB0", commPtr,
                &edgeDevice, "PT1");
        edgeDevice.addSensor(sensorPtr);

//        sensorPtr = new TemperatureSensor("/dev/pts/2", commPtr,
//                        &edgeDevice, "TT1");
//                edgeDevice.addSensor(sensorPtr);
    }

    commPtr->connect();
    commPtr->subscribe();
    edgeDevice.setCommunicator(commPtr);
    edgeDevice.runForever();

	return 0;
}
