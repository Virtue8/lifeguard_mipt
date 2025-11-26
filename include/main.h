#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MAX30105.h"

#include "analysis.h"
#include "constants.h"
#include "connection.h"
#include <malloc.h>

MAX30105 particleSensor;

#define debug Serial 

void make_buzz ();
void smart_delay (int time);

//--------DATA STRUCT----------------

struct DeviceData
{
    //--------Optic Sensor-----------
    float   pulse_period = 0;
    int     pulse_status = OK;
    bool    pulse_analysis = false;
    bool    is_worn      = false;
    hw_timer_t* sensor_timer = NULL;
    
    //----------Button--------------- 
    bool    button_state = false;
    
    //---------Accelerometer---------
    MPU6050 mpu;
    bool    is_moving    = false;
    bool    movement_analysis = false;

    //------------Battery------------
    int     battery_status   = 100;
    bool    battery_is_low      = false;
    hw_timer_t* battery_timer = NULL;

    //------------Buzzer-------------
    hw_timer_t* buzzer_timer = NULL;
    bool        is_buzzing = false;

};

#endif
