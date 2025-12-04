#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

#include "main.h"
#include "constants.h"
#include "connection.h"
#include "analysis.h"

MAX30105 particleSensor;
MPU6050 mpu;

void IRAM_ATTR check_movement();
void IRAM_ATTR check_battery();
void IRAM_ATTR make_buzz();

void button_interrupt();
void start_buzzing();
void end_buzzing();
void emergency();
void check_button();
void enter_deep_sleep();


unsigned long last_measure = 0;

#define debug Serial

hw_timer_t* sensor_timer = NULL;
hw_timer_t* battery_timer = NULL;
hw_timer_t* buzzer_timer = NULL;

bool button_was_pressed = false;
bool movement_is_checking = false;
bool pulse_is_checking = false;
bool is_buzzing = false;

void setup()
{
    debug.begin(9600);
    debug.println("MAX30105 Basic Readings Example");

    // Initialize sensor
    if (particleSensor.begin() == false)
    {
      debug.println("MAX30105 was not found. Please check wiring/power. ");
      while (1);
    }

    particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

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
    //timerAlarmEnable(buzzer_timer);//TODO: вкл/выкл при необходимости
    timerAlarmDisable(buzzer_timer);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRO_PIN, OUTPUT);

    pinMode(BATTERY_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(LED_PIN, HIGH);

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        unsigned long press_start = millis();
        while (digitalRead(BUTTON_PIN) == LOW)
            if (millis() - press_start > 3000)
                break;
        if (millis() - press_start > 3000)
            Serial.println("Просыпаемся");
        else
            enter_deep_sleep();
    }

    attachInterrupt(BUTTON_PIN, button_interrupt, FALLING);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    mpu.initialize();

    // if (connectToWiFi())
    //     send_tg_message("Connection established");
    connect_to_wifi();
}

#define SAMPLE_COUNT 1000  // 10 сек при 10 ms dt

Sample buffer[SAMPLE_COUNT];
int buffer_index = 0;

void loop() 
{
    if (button_was_pressed)
    {
        unsigned long press_start = millis();
        while (digitalRead(BUTTON_PIN) == LOW)
        {
            Serial.println("Ждем");
            if (millis() - press_start > 3000)
                enter_deep_sleep();
        }
        Serial.println("Ложная тревога");
        button_was_pressed = false;
    }

    long irValue = particleSensor.getIR();
    buffer[buffer_index].value = irValue;
    buffer[buffer_index].time_ms = millis();
    buffer_index++;

    if (buffer_index >= SAMPLE_COUNT) {
        is_pulsing(buffer, SAMPLE_COUNT);
        buffer_index = 0; // новая серия 10 секунд
    }

    delay(10); // dt = 10ms
    
}


/*
void loop()
{
<<<<<<< HEAD
    if (movement_is_checking)
    {
        if (!is_movement())
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
}*/

void check_button() {
  /*static unsigned long last_press = 0;
  const unsigned long LONG_PRESS_TIME = 3000; // 3 секунды

  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long press_start = millis();

    // Ждем, пока кнопка отпущена или прошло время
    while (digitalRead(BUTTON_PIN) == LOW) {
      if (millis() - press_start > LONG_PRESS_TIME) {
        // Долгое нажатие обнаружено
        Serial.println("Long press detected - disabling device");

        // Индикация
        digitalWrite(LED_PIN, LOW);
        delay(1000);
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);

        enter_deep_sleep();
        return;
      }
      delay(50);
    }

    // Короткое нажатие (можно использовать для других функций)
    if (millis() - press_start < 1000) {
      Serial.println("Short press");
      // Например, показать статус батареи
    }
  }*/
}

void enter_deep_sleep()
{
    Serial.println("Спим");
    Serial.flush();

    // Настройка пробуждения по кнопке (GPIO0)
    esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0); // 0 = LOW при нажатии

    // Можно также настроить пробуждение по таймеру
    // esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 секунд

    // Отключаем всё, что можно
    disconnect_wifi();
    button_was_pressed = false;

    // Переводим в глубокий сон
    esp_deep_sleep_start();
}

// Функция для принудительного пробуждения (если нужно)
void force_wakeup() {
  // Можно использовать touch-пробуждение
  touchAttachInterrupt(T0, [](){
    // Пустая функция, просто пробуждаем
  }, 40);
  esp_sleep_enable_touchpad_wakeup();
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
