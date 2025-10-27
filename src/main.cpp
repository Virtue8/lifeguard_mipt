#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include "main.h"
#include "constants.h"
#include "connection.h"
#include "analysis.h"

void IRAM_ATTR check_movement();
void IRAM_ATTR check_battery();
void emergency();

MPU6050 mpu;

hw_timer_t* sensor_timer = NULL;
hw_timer_t* battery_timer = NULL;
hw_timer_t* buzzer_timer = NULL;

bool button_pressed = true;
bool movement_is_checking = false;
bool pulse_is_checking = false;
bool is_buzzing = false;

void setup()
{
    Serial.begin(9600);

    //инициализация таймера проверки датчика
    sensor_timer = timerBegin(0, 80, true); //инициализация таймера. 80 - коэф. предделителя
    timerAttachInterrupt(sensor_timer, &check_movement, true); //прикрепить функцию к таймеру
    timerAlarmWrite(sensor_timer, SENSOR_CHECK_PERIOD, true); //задать время в мкс
    timerAlarmEnable(sensor_timer); //включить таймер

    //инициализация таймера проверки батареи
    battery_timer = timerBegin(1, 80, true);
    timerAttachInterrupt(battery_timer, &check_battery, true);
    timerAlarmWrite(battery_timer, BATTERY_CHECK_PERIOD, true);
    timerAlarmEnable(battery_timer);

    //инициализация таймера для пищалки
    buzzer_timer = timerBegin(2, 80, true);
    timerAttachInterrupt(buzzer_timer, &make_buzz, true);
    timerAlarmWrite(buzzer_timer, 500000, true);
    timerAlarmEnable(buzzer_timer);//TODO: вкл/выкл при необходимости

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRO_PIN, OUTPUT);

    pinMode(BATTERY_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    mpu.initialize();

    // if (connectToWiFi())
    //     send_tg_message("Connection established");
    connectToWiFi();
}

void loop()
{
    if (movement_is_checking)
    {
        if (!is_movement())
        {
            bool b = false;
            for (int i = 0; i < 5 && !b; i++)
            {
                b |= is_movement();
                Serial.println(b);
            }
            if (!b)
            {
                send_tg_message("NO MOVEMENT");
            }
        }
    }
    // if (movement_is_checking)
    // {
    //     if (!is_movement())
    //     {
    //         int i = 1;
    //         while (i <= 5 || !is_movement)
    //             i++;
    //         if (i == 5)
    //         {
    //             is_buzzing = true;
    //             int start_time = millis();
    //             while (!button_pressed && millis() - start_time < 10000)
    //                 delay(10);
    //             if (millis() - start_time < 10000)
    //                 is_buzzing = false;
    //             else
    //                 emergency();
    //         }
    //     }
    // }
}

void emergency()
{
    send_tg_message("NO MOVEMENT");
    while (!button_pressed)
        ;
    is_buzzing = false;
    movement_is_checking = false;
    pulse_is_checking = false;
}

void make_buzz()
{
    if (is_buzzing)
    {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        digitalWrite(VIBRO_PIN, !digitalRead(VIBRO_PIN));
    }
    else
    {
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(VIBRO_PIN, LOW);
    }
}

void smart_delay(int time)
{
    int start = millis();
    while (millis() - start < time)
        ;
}

void IRAM_ATTR check_movement()
{
    movement_is_checking = true;
}

void IRAM_ATTR check_battery()
{
    const int FULL_CHARGE_V = 4;
    const int LOW_CHARGE_V = 3;
    const int V_DIVIDER = 2;

    float V_sum = 0;

    for(int i = 0; i < 100000; i++)
        V_sum = (float)analogRead(BATTERY_PIN)/4096*3.3*V_DIVIDER;

    Serial.println(V_sum);
    if (V_sum < LOW_CHARGE_V)
        Serial.println("LOW CHARGE");
}
