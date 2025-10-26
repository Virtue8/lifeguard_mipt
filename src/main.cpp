#include "Arduino.h"
#include "constants.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

void IRAM_ATTR check_movement();
void IRAM_ATTR check_battery();
void make_buzz();
void smart_delay(int time);

MPU6050 mpu;

hw_timer_t* sensor_timer = NULL;
hw_timer_t* battery_timer = NULL;
hw_timer_t* buzzer_timer = NULL;

bool global_is_checking = false;
bool global_buzzing = true;

bool check_pulse()
{
    return true;
}

bool is_movement()
{
    //return true;
    mpu.setSleepEnabled(false);

    int N = 100;

    struct single_data
    {
        int16_t ax;
        int16_t ay;
        int16_t az;
    };
    struct single_data data[N];

    int min_ax = 32767;
    int max_ax = -32768;
    for (int i = 0; i < N; i++)
    {
        int16_t ax, ay, az;
        for (int j = 0; j < 10; j++)
        {
            mpu.getAcceleration(&ax, &ay, &az);
            data[i].ax += ax;
            data[i].ay += ay;
            data[i].az += az;
            smart_delay(5);
        }
        data[i].ax /= 10;
        data[i].ay /= 10;
        data[i].az /= 10;

        if (data[i].ax > max_ax)
            max_ax = data[i].ax;
        if (data[i].ax < min_ax)
            min_ax = data[i].ax;
    }
    Serial.println(max_ax - min_ax);

    return true;
}

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
    timerAlarmEnable(buzzer_timer);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRO_PIN, OUTPUT);

    pinMode(BATTERY_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    mpu.initialize();
}

void loop()
{
    is_movement();
}

void make_buzz()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    if (global_buzzing)
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

void IRAM_ATTR check_movement()
{
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    if (!global_is_checking)
        if (true)
            check_pulse();
    digitalWrite(LED_PIN, LOW);
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

void smart_delay(int time)
{
    int start = millis();
    while (millis() - start < time)
        ;
}
