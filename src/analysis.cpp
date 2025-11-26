#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#include "main.h"
#include "analysis.h"
#include "constants.h"

extern MPU6050 mpu;
extern bool movement_is_checking;
extern MAX30105 PulseSensor;

bool is_pulsing ()
{
    Serial.println ("MAX30105 Heart Rate Monitor Started");

    // Инициализация сенсора
    if (PulseSensor.begin () == false)
    {
        Serial.println ("MAX30105 was not found. Please check wiring/power.");
        return false;
    }

    PulseSensor.setup (); // Конфигурация сенсора
    PulseSensor.setPulseAmplitudeRed (0x0A); // Настройка амплитуды для лучшего сигнала
    PulseSensor.setPulseAmplitudeIR (0x0A);

    // Параметры для анализа пульса
    const int SAMPLE_WINDOW = 10000; // 10 секунд измерения
    const int MIN_HEART_RATE = 40;   // минимальный нормальный пульс
    const int MAX_HEART_RATE = 180;  // максимальный нормальный пульс
    const int IR_THRESHOLD = 50000;  // порог для обнаружения пульсации

    unsigned long startTime = millis ();
    int samples = 0;
    int pulseCount = 0;
    int lastIRValue = 0;
    bool wasRising = false;
    long totalIR = 0;

    Serial.println ("Starting heart rate monitoring...");

    while (millis () - startTime < SAMPLE_WINDOW)
    {
        int irValue = PulseSensor.getIR ();
        totalIR += irValue;
        samples++;

        // Обнаружение пиков (простой алгоритм)
        if (irValue > IR_THRESHOLD)
        {
            if (lastIRValue <= IR_THRESHOLD && !wasRising)
            {
                // Обнаружен восходящий фронт - потенциальный удар сердца
                pulseCount++;
                wasRising = true;
                Serial.println ("Beat detected!");
            }
        }
        else
        {
            wasRising = false;
        }

        lastIRValue = irValue;

        // Вывод данных для отладки (можно уменьшить частоту)
        if (samples % 100 == 0) {
            Serial.print ("IR: ");
            Serial.print (irValue);
            Serial.print (" | Beats: ");
            Serial.println (pulseCount);
        }

        delay (10);
    }

    // Расчет среднего значения IR и пульса
    int averageIR = totalIR / samples;
    int heartRate = (pulseCount * 60000) / SAMPLE_WINDOW; // ударов в минуту

    Serial.println ("\n=== Monitoring Results ===");
    Serial.print ("Total samples: ");
    Serial.println (samples);
    Serial.print ("Average IR: ");
    Serial.println (averageIR);
    Serial.print ("Detected heart rate: ");
    Serial.print (heartRate);
    Serial.println (" BPM");

    // Анализ результатов
    bool isNormal = true;

    if (averageIR < 10000)
    {
        Serial.println ("ALERT: Poor sensor contact - check placement!");
        isNormal = false;
    }
    else if (pulseCount == 0)
    {
        Serial.println ("ALERT: No heartbeat detected!");
        isNormal = false;
    }
    else if (heartRate < MIN_HEART_RATE)
    {
        Serial.println ("ALERT: Bradycardia detected - heart rate too low!");
        isNormal = false;
    }
    else if (heartRate > MAX_HEART_RATE)
    {
        Serial.println ("ALERT: Tachycardia detected - heart rate too high!");
        isNormal = false;
    }
    else
    {
        Serial.println ("Heart rate is within normal range.");
    }

    // Визуальная индикация (можно заменить на светодиод/вибрацию)
    if (!isNormal)
    {
        // Сигнализация - мигание или вибрация
        for (int i = 0; i < 3; i++)
        {
            Serial.println ("ALARM!");
            delay (500);
        }
    }

    return isNormal;
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
