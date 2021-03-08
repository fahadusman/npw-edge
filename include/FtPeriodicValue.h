/*
 * FtPeriodicValue.h
 *
 *  Created on: Mar 3, 2021
 *      Author: Fahad Usman
 */

#ifndef FTPERIODICVALUE_H_
#define FTPERIODICVALUE_H_

#include <PeriodicValue.h>

class FtPeriodicValue: public PeriodicValue {
protected:
    double flowVelociy, volumeFlow, massFlow, temperature, density;
    double totaliser1Value, totaliser2Value;
    uint16_t * eventGroups, eventGroupsLen;
//    int sensorStatus;

public:
    FtPeriodicValue(double fv, double vf, double mf, double temp, double d,
            double t1v, double t2v, uint16_t *eg, uint16_t egLen, uint64_t time,
            const char *id, int s);

    std::string serializeJson() override;
    unsigned char * serialize(int & length) override;
    int deserialize(const unsigned char * serialBuff, const int & len);
    size_t getSerializedBuffLen() override;


    virtual ~FtPeriodicValue();
};

#endif /* FTPERIODICVALUE_H_ */
