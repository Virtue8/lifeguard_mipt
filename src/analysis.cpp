#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"

#include "main.h"
#include "analysis.h"
#include "constants.h"

extern bool movement_is_checking;

extern MAX30105 particleSensor;
extern MPU6050 mpu;

#define MAX_EXTREMUMS 100
#define MIN_AMPLITUDE 5     // минимальная разница между экстремумами
#define WINDOW_SIZE 25      // размер окна для локального экстремума (примерно 1/3-1/2 периода)
#define SAMPLE_COUNT 1000   // 10 секунд при dt = 10ms

#define MIN_HEARTRATE 100
#define MAX_HEARTRATE 200

#define MIN_AVG_VALUE 70000
#define MAX_AVG_VALUE 120000

bool is_pulsing(Sample *samples, int n)
{
    Serial.println("Analysing pulse");
    int extremums[MAX_EXTREMUMS];
    int count = 0;
    int sum_value = 0;

    for (int i = WINDOW_SIZE; i < n - WINDOW_SIZE; i++) {
        bool is_peak = true;
        bool is_trough = true;

        for (int j = i - WINDOW_SIZE; j <= i + WINDOW_SIZE; j++) {
            if (j == i) continue;
            if (samples[i].value <= samples[j].value) is_peak = false;
            if (samples[i].value >= samples[j].value) is_trough = false;
        }

        if ((is_peak || is_trough) &&
            (count == 0 ||
             fabs(samples[i].value - samples[extremums[count-1]].value) >= MIN_AMPLITUDE))
        {
            extremums[count++] = i;
            if (count >= MAX_EXTREMUMS) break;
        }
    }

    // --- FIX HERE: not enough extremums to compute frequency ---
    if (count < 2) {
        Serial.println("Недостаточно экстремумов для вычисления частоты.");
        return false;
    }

    // статистика
    float sum_intervals = 0;
    for (int i = 1; i < count; i++) {
        uint32_t dt = samples[extremums[i]].time_ms -
                      samples[extremums[i-1]].time_ms;
        sum_intervals += dt;
        sum_value += samples[i].value;
    }

    int avg_value = sum_value / (count - 1);
    float avg_interval = sum_intervals / (count - 1); // мс
    float freq_per_min = 60000.0 / avg_interval;

    Serial.print("Количество экстремумов: "); Serial.println(count);
    Serial.print("Среднее время между экстремумами (мс): "); Serial.println(avg_interval);
    Serial.print("Среднее значение экстремумов: "); Serial.println(avg_value);
    Serial.print("Частота экстремумов в минуту: "); Serial.println(freq_per_min);

    Serial.println("Экстремумы:");
    for (int i = 1; i < count; i++) {
        int idx_prev = extremums[i-1];
        int idx_curr = extremums[i];
        float derivative =
            (samples[idx_curr].value - samples[idx_prev].value) /
            ((samples[idx_curr].time_ms - samples[idx_prev].time_ms) / 1000.0f);

        Serial.print("Экстремум #"); Serial.print(i);
        Serial.print(" (индекс "); Serial.print(idx_curr); Serial.print(")");
        Serial.print(", значение: "); Serial.print(samples[idx_curr].value);
        Serial.print(", производная: "); Serial.println(derivative);
    }

    if (freq_per_min < MIN_HEARTRATE) {Serial.print("Крыжовник"); return false;}
    if (freq_per_min > MAX_HEARTRATE) {Serial.print("Крыжовник"); return false;}

    if (avg_value < MIN_AVG_VALUE) {Serial.print("Крыжовник"); return false;}
    if (avg_value > MAX_AVG_VALUE) {Serial.print("Крыжовник"); return false;}

    return true;
}


bool is_moving()
{
    Serial.println("checking gyros");
    mpu.setSleepEnabled(false);
    delay(5);

    int N = 100;

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
    Serial.print(max_ax - min_ax);
    Serial.print(" ");
    Serial.print(max_ay - min_ay);
    Serial.print(" ");
    Serial.println((max_ax - min_ax >= 3500) || (max_ay - min_ay >= 3500));
    return (max_ax - min_ax >= 3500) || (max_ay - min_ay >= 3500);// || (max_az - min_az >= 2500);
}
