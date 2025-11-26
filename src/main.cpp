#include "main.h"
#include "constants.h"
#include "connection.h"
#include "analysis.h"

MPU6050 mpu;

hw_timer_t* sensor_timer = NULL;
hw_timer_t* battery_timer = NULL;
hw_timer_t* buzzer_timer = NULL;

bool button_pressed = false;
bool movement_is_checking = false;
bool pulse_is_checking = false;
bool is_buzzing = false;

DeviceData data;

void setup(struct DeviceData * data)
{
    Serial.begin (9600);

    //инициализация таймера проверки датчика
    data->sensor_timer = timerBegin (0, 80, true); //инициализация таймера. 80 - коэф. предделителя

    timerAttachInterrupt (data->sensor_timer, movement_check, true); //прикрепить функцию к таймеру
    timerAlarmWrite (data->sensor_timer, SENSOR_CHECK_PERIOD, true); //задать время в мкс
    timerAlarmEnable (data->sensor_timer); //включить таймер

    //инициализация таймера проверки батареи
    data->battery_timer = timerBegin (1, 80, true); 
    
    timerAttachInterrupt (data->battery_timer, &battery_check, true);
    timerAlarmWrite (data->battery_timer, BATTERY_CHECK_PERIOD, true);
    timerAlarmEnable (data->battery_timer);

    //инициализация таймера для пищалки
    buzzer_timer = timerBegin(2, 80, true);
    timerAttachInterrupt(buzzer_timer, make_buzz, true);
    timerAlarmWrite(buzzer_timer, 500000, true);
    timerAlarmEnable(buzzer_timer);//TODO: вкл/выкл при необходимости
    //timerAlarmDisable(buzzer_timer);

    pinMode (LED_PIN, OUTPUT);
    pinMode (BUZZER_PIN, OUTPUT);
    pinMode (VIBRO_PIN, OUTPUT);

    pinMode (BATTERY_PIN, INPUT_PULLUP);
    pinMode (SDA_PIN, INPUT_PULLUP);
    pinMode (SCL_PIN, INPUT_PULLUP);
    pinMode (BUTTON_PIN, INPUT);

    attachInterrupt(BUTTON_PIN, button_interrupt, HIGH);

    Wire.begin (SDA_PIN, SCL_PIN);
    Wire.setClock (400000);
    data->mpu.initialize ();

    if (connect_to_wifi ()) send_tg_message ("Connection established");
    connect_to_wifi ();
}

void loop()
{
    if (movement_check)
    {
        if (!is_moving ())
        {
            bool b = false;
            for (int i = 0; i < 5 && !b && !button_pressed; i++)
            {
                b |= is_moving ();
                Serial.print("b = ");
                Serial.print(b);
                Serial.print("; button = ");
                Serial.println(button_pressed);
            }
            if (!b && !button_pressed)
            {
                send_tg_message("NO MOVEMENT");
            }
        }
    }
    // if (movement_check)
    // {
    //     if (!is_moving())
    //     {
    //         int i = 1;
    //         while (i <= 5 || !is_movement)
    //             i++;
    //         if (i == 5)
    //         {
    //             is_buzzing = true;
    //             int start_time = millis();
    //             while (!button_was_pressed && millis() - start_time < 10000)
    //                 delay(10);
    //             if (millis() - start_time < 10000)
    //                 is_buzzing = false;
    //             else
    //                 emergency();
    //         }
    //     }
    // }
}
