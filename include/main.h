#ifndef MAIN_H
#define MAIN_H

#include "MAX30105.h"
#include "MPU6050.h"

extern MAX30105 particleSensor;
extern MPU6050 mpu;

void make_buzz();
void smart_delay(int time);

#endif
