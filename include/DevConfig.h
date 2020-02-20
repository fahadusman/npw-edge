/*
 * DevConfig.h
 *
 *  Created on: Dec 20, 2019
 *      Author: Fahad Usman
 */

#ifndef INCLUDE_DEVCONFIG_H_
#define INCLUDE_DEVCONFIG_H_

#include <stdint.h>

struct DevConfig {
    int32_t min, max, def;
};

const DevConfig kDcNpwNumPack       { 1, 50, 5 },
        kDcNpwExpiryTime      { 1, 600, 60 },
        kDcMaxTimePeriodic    { 5, 300, 10 },
        kDcMinTimePeriodic    { 1, 60, 15 },
        kDcOnChangThshPt     { 0, 2000, 1999 },
        kDcOnChangThshTt     { 50, 2000, 100 },
        kDcSampleIntervalNpw     { 10, 1000, 20 },
        kDcNumSamples1stAvg    { 1, 10000, 150 },
        kDcNumSamples2ndAvg    { 1, 10000, 1000 },
        kDcStartSample2ndAvg   { 0, 500, 250 },
        kDcNpwPtThsh          { 1, 2000, 4 },
        kDcNpwSampleBefore    { 1, 1000, 150 },
        kDcNpwSampleAfter     { 1, 1000, 350 },
        kDcTestFlag            { 0, 1, 0 },
        kDcRebootTime          { -1, 10, -1 },
        kDcHeartbeatInterval   { 10, 3600, 600 };



#endif /* INCLUDE_DEVCONFIG_H_ */
