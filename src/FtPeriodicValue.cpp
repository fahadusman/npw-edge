/*
 * FtPeriodicValue.cpp
 *
 *  Created on: Mar 3, 2021
 *      Author: Fahad Usman
 */

#include "glog/logging.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "FtPeriodicValue.h"

FtPeriodicValue::FtPeriodicValue(double fv, double vf, double mf, double temp,
        double d, double t1v, double t2v, uint16_t *eg, uint16_t egLen,
        uint64_t time, const char *id, int s) :
        PeriodicValue(mf, time, id, s) {

    flowVelociy = fv;
    volumeFlow = vf;
    massFlow = mf;
    temperature = temp;
    density = d;
    totaliser1Value = t1v;
    totaliser2Value = t2v;
    eventGroups = new uint16_t[egLen];
    eventGroupsLen = egLen;
    for (int i = 0; i < eventGroupsLen; i++) {
        eventGroups[i] = eg[i];
    }

    timeStamp = time;
    strncpy(sensorId, id, sensorIdLen);
    sensorStatus = s;


}

std::string FtPeriodicValue::serializeJson() {
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();
    writer.Key(sensorId);
    writer.StartObject();
    writer.Key("flowVelocity");
    writer.Double(flowVelociy);
    writer.Key("operVolumeFlow");
    writer.Double(volumeFlow);
    writer.Key("massFlow");
    writer.Double(massFlow);
    writer.Key("temperature");
    writer.Double(temperature);
    writer.Key("operDensity");
    writer.Double(density);
    writer.Key("totaliser1Value");
    writer.Double(totaliser1Value);
    writer.Key("totaliser2Value");
    writer.Double(totaliser2Value);
    writer.Key("eventGroups");
    writer.StartArray();
    for (int i = 0; i < eventGroupsLen; i++) {
        writer.Uint(eventGroups[i]);
    }
    writer.EndArray();
    writer.EndObject();

    writer.Key((std::string("t_") + sensorId).c_str());
    writer.Uint64(timeStamp);
    writer.Key((std::string("q_") + sensorId).c_str());
    writer.Int(sensorStatus);
    writer.EndObject();

    return s.GetString();
}

unsigned char * FtPeriodicValue::serialize(int & length) {
    LOG(ERROR) << "FtPeriodicValue::serialize method not implemented";
    length = 0;
    return nullptr;
}

int FtPeriodicValue::deserialize(const unsigned char * serialBuff, const int & len) {
    LOG(ERROR) << "FtPeriodicValue::deserialize method not implemented";
    if (serialBuff and len) {
        return 0;
    }
    return len;
}

size_t FtPeriodicValue::getSerializedBuffLen() {
    LOG(ERROR) << "FtPeriodicValue::getSerializedBuffLen method not implemented";
    return 0;
}

FtPeriodicValue::~FtPeriodicValue() {
    delete eventGroups;
}

