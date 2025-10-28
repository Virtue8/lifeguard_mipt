#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include "main.h"
#include "analysis.h"
#include "constants.h"

extern MPU6050 mpu;
extern bool movement_is_checking;

bool is_pulse()
{
    return true;
}

bool is_movement()
{
    digitalWrite(LED_PIN, HIGH);
    mpu.setSleepEnabled(false);
    delay(5);

    int N = 200;

    struct single_data
    {
        int ax;
        int ay;
        int az;
    };
    struct single_data data[N] = {0};

    int min_ax = 32767;
    int max_ax = -32768;
    int min_ay = 32767;
    int max_ay = -32768;
    int min_az = 32767;
    int max_az = -32768;
    for (int i = 0; i < N; i++)
    {
        int16_t ax, ay, az;
        //mpu.getAcceleration(&data[i].ax, &data[i].ay, &data[i].az);
        for (int j = 0; j <= 10; j++)
        {
            mpu.getAcceleration(&ax, &ay, &az);
            data[i].ax += ax;
            data[i].ay += ay;
            data[i].az += az;
        }
        data[i].ax /= 10;
        data[i].ay /= 10;
        data[i].az /= 10;
        // Serial.print(data[i].ax);
        // Serial.print(" ");
        // Serial.print(max_ax - min_ax);
        // Serial.print(" ");
        // Serial.print(max_ay - min_ay);
        // Serial.print(" ");
        // Serial.println(max_az - min_az);
        if (data[i].ax > max_ax)
            max_ax = data[i].ax;
        if (data[i].ax < min_ax)
            min_ax = data[i].ax;

        if (data[i].ay > max_ay)
            max_ay = data[i].ay;
        if (data[i].ay < min_ay)
            min_ay = data[i].ay;

        if (data[i].az > max_az)
            max_az = data[i].az;
        if (data[i].az < min_az)
            min_az = data[i].az;
        delay(50);
    }

    movement_is_checking = false;
    mpu.setSleepEnabled(true);
    digitalWrite(LED_PIN, LOW);
    Serial.print(max_ax - min_ax);
    Serial.print(" ");
    Serial.print(max_ay - min_ay);
    Serial.print(" ");
    Serial.println((max_ax - min_ax >= 4000) || (max_ay - min_ay >= 4000));
    return (max_ax - min_ax >= 4000) || (max_ay - min_ay >= 4000);// || (max_az - min_az >= 2500);
}
