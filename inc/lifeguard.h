#ifndef LIFEGUARD_H
#define LIFEGUARD_H

//----------- INCLUDES ------------//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_timer.h"

static const char *TAG = "BRACELET_2";

//---------- PIN CONFIG -----------//

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz

#define VIBRO_PIN          (gpio_num_t) 26
#define BUZZER_PIN         (gpio_num_t) 25
#define BUTTON_PIN         (gpio_num_t) 0   // user acknowledge
#define LED_PIN            (gpio_num_t) 2
#define GSM_TX_PIN         (gpio_num_t) 17
#define GSM_RX_PIN         (gpio_num_t) 16
#define BAT_ADC_CHANNEL    ADC1_CHANNEL_6   // GPIO34 (example)
#define WORN_CAP_PIN       (gpio_num_t) 33  // optional

//--------- I2C addresses ----------//

#define MPU6050_ADDR        0x68
#define MAX30102_ADDR       0x57

//-------- Parameters (tune) -------//

#define INACTIVITY_WINDOW_MS   (5 * 60 * 1000)
#define MOTION_SAMPLE_MS       200
#define MOTION_RMS_THRESHOLD   0.08f

#define HR_SAMPLING_RATE_HZ    100            // target sample rate for MAX30102 in this simple code
#define HR_CHECK_DURATION_MS   (15 * 1000)
#define HR_LOW_BPM             40
#define HR_HIGH_BPM            140
#define ACK_TIMEOUT_MS         (30 * 1000)

#define BATTERY_LOW_VOLTAGE    3.5f

//---------- Event queue -----------//

enum event_t 
{
    EVT_MOTION,
    EVT_NO_MOTION,
    EVT_BATT_LOW,
    EVT_BATT_OK,
    EVT_WORN,
    EVT_NOT_WORN,
    EVT_HR_ABNORMAL,
    EVT_HR_OK,
    EVT_EMERGENCY
};

static QueueHandle_t evt_queue = NULL;

//------ Inline Functions ------//

static inline void vibro_on   ()    { gpio_set_level(VIBRO_PIN, 1); }
static inline void vibro_off  ()    { gpio_set_level(VIBRO_PIN, 0); }
static inline void buzzer_on  ()    { gpio_set_level(BUZZER_PIN, 1); }
static inline void buzzer_off ()    { gpio_set_level(BUZZER_PIN, 0); }

//------ Function Prototypes ------//

// Init functions
esp_err_t gpio_init_all (void);
esp_err_t i2c_master_init (void);
static esp_err_t mpu6050_init (void);
static esp_err_t max30102_init (void);

// Task functions
static void monitor_task(void *pv);
static void gsm_task (void *pv);

// Core lifeguard functions
void lifeguard_check_battery (void);
void lifeguard_check_wear_status (void);
void lifeguard_check_inactivity (void);
void lifeguard_check_heartbeat (void);

// Emergency / notifications
void lifeguard_notify_user (void);
void lifeguard_emergency_mode (void);

#endif