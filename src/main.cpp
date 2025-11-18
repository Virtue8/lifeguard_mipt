#include "main.h"
#include "constants.h"
#include "connection.h"
#include "analysis.h"

void IRAM_ATTR check_movement();
void IRAM_ATTR check_battery();
void IRAM_ATTR make_buzz();

void button_interrupt();
void start_buzzing();
void end_buzzing();
void emergency();

MPU6050 mpu;

hw_timer_t* sensor_timer = NULL;
hw_timer_t* battery_timer = NULL;
hw_timer_t* buzzer_timer = NULL;

bool button_was_pressed = false;
bool movement_is_checking = false;
bool pulse_is_checking = false;
bool is_buzzing = false;

void setup()
{
    Serial.begin (9600);

    //инициализация таймера проверки датчика
    data.sensor_timer = timerBegin (0, 80, true); //инициализация таймера. 80 - коэф. предделителя

    timerAttachInterrupt (data.sensor_timer, (void IRAM_ATTR(*)()) movement_check, true); //прикрепить функцию к таймеру
    timerAlarmWrite (data.sensor_timer, SENSOR_CHECK_PERIOD, true); //задать время в мкс
    timerAlarmEnable (data.sensor_timer); //включить таймер

    //инициализация таймера проверки батареи
    data.battery_timer = timerBegin (1, 80, true); 
    
    timerAttachInterrupt (data.battery_timer, (void IRAM_ATTR(*)()) battery_check, true);
    timerAlarmWrite (data.battery_timer, BATTERY_CHECK_PERIOD, true);
    timerAlarmEnable (data.battery_timer);

    //инициализация таймера для пищалки
    buzzer_timer = timerBegin(2, 80, true);
    timerAttachInterrupt(buzzer_timer, &make_buzz, true);
    timerAlarmWrite(buzzer_timer, 500000, true);
    //timerAlarmEnable(buzzer_timer);//TODO: вкл/выкл при необходимости
    timerAlarmDisable(buzzer_timer);

    pinMode (LED_PIN, OUTPUT);
    pinMode (BUZZER_PIN, OUTPUT);
    pinMode (VIBRO_PIN, OUTPUT);

    pinMode(BATTERY_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT);

    attachInterrupt(BUTTON_PIN, button_interrupt, HIGH);

    Wire.begin (SDA_PIN, SCL_PIN);
    Wire.setClock (400000);
    data.mpu.initialize ();

    if (connect_to_wifi ()) send_tg_message ("Connection established");
    connect_to_wifi ();
}

void loop()
{
    if (movement_check)
    {
        if (!is_moving (&data))
        {
            bool b = false;
            for (int i = 0; i < 5 && !b && !button_was_pressed; i++)
            {
                b |= is_movement();
                Serial.print("b = ");
                Serial.print(b);
                Serial.print("; button = ");
                Serial.println(button_was_pressed);
            }
            if (!b && !button_was_pressed)
            {
                start_buzzing();
                int start_time = millis();
                while (millis() - start_time < 10000 && !button_was_pressed)
                    delay(100);
                if (button_was_pressed)
                {
                    end_buzzing();
                    button_was_pressed = false;
                }
                else
                {
                    emergency();
                }
            }
            if (button_was_pressed)
                button_was_pressed = false;
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

void button_interrupt()
{
    button_was_pressed = true;
    //Serial.println("button!");
}

void emergency()
{
    send_tg_message("NO MOVEMENT");
    while (!button_was_pressed)
    {
        delay(100);
        /*Serial.print("waiting, button = ");
        Serial.println(button_was_pressed);*/
    }
    end_buzzing();
    is_buzzing = false;
    movement_is_checking = false;
    pulse_is_checking = false;
    button_was_pressed = false;
}

void start_buzzing()
{
    timerAlarmEnable(buzzer_timer);
}

void end_buzzing()
{
    timerAlarmDisable(buzzer_timer);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(VIBRO_PIN, LOW);
}

void make_buzz()
{
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    digitalWrite(VIBRO_PIN, !digitalRead(VIBRO_PIN));
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
