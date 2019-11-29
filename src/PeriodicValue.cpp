/*
 * PeriodicValue.cpp
 *
 *  Created on: Nov 28, 2019
 *      Author: Fahad Usman
 */

#include "PeriodicValue.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

std::string PeriodicValue::serializeJson() {
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> writer(s);

    writer.StartObject();
    writer.Key("id");
    writer.String(sensorId.c_str());
    writer.Key("v");
    writer.Double(sensorValue);
    writer.Key("t");
    writer.Uint64(timeStamp);
    writer.EndObject();

    return s.GetString();
}

void * PeriodicValue::serialize(int & length) {
    return NULL;
}
