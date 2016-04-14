#ifndef _GY85_H
#define _GY85_H

#include <wirish/wirish.h>
#include <i2c.h>

// Raw values
struct gy85_value
{
    short acc_x, acc_y, acc_z;
    short gyro_x, gyro_y, gyro_z;
    short magn_x, magn_y, magn_z;
};

// Initializing
void gy85_init(i2c_dev *dev);
void gy85_update(i2c_dev *dev, struct gy85_value *value);

#endif
