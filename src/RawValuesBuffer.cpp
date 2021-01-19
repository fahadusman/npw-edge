/*
 * RawValuesBuffer.cpp
 *
 *  Created on: Jan 12, 2021
 *      Author: Fahad Usman
 */

#include <sqlite_modern_cpp.h>
#include <chrono>
#include <glog/logging.h>

#include <RawValuesBuffer.h>
RawValuesBuffer::RawValuesBuffer() {
    timeStamp = 0;
    samplingIntervalMs = 20;
}

RawValuesBuffer::RawValuesBuffer(const uint64_t &t, const std::string &sId,
        const unsigned int & sInterval, const std::vector<double> &l) {
    timeStamp = t;
    sensorId = sId;
    valuesList = l;
    samplingIntervalMs = sInterval;
}

RawValuesBuffer::~RawValuesBuffer() {
}

void RawValuesBuffer::storeInDatabase(uint64_t maxDurationHr) {
    uint64_t maxDurationMs = maxDurationHr*60*60*1000;
    try {
        sqlite::database rawDumpDb("./" + sensorId + ".db");

        rawDumpDb
                << "CREATE TABLE IF NOT EXISTS raw_pt_values (s_time INT PRIMARY KEY NOT NULL, s_interval INT NOT NULL, values_array BLOB NOT NULL);\n";

        rawDumpDb << "INSERT INTO raw_pt_values VALUES (?, ?, ?)" << timeStamp
                << samplingIntervalMs << valuesList;
        uint64_t oldestRecordTimeMs = std::chrono::duration_cast<
                std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()
                - maxDurationMs;
        rawDumpDb << "DELETE FROM raw_pt_values where s_time < ?"
                << oldestRecordTimeMs;
    } catch (const std::exception &e) {
        LOG(ERROR) << " Exception in storing rawBuffer to DB: " << e.what();
    }

}
