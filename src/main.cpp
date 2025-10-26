// main.c
// ESP-IDF example: MPU6050 (accel) + MAX30102 (HR) + SIM800-style GSM skeleton
// Single-file scaffold. Tune thresholds and timings on hardware.
//
// Notes:
// - Replace phone numbers in gsm_send_sms/gsm_call_number
// - MAX30102 and MPU6050 code here is minimal and intended to be integrated/tested.
// - This code compiles under ESP-IDF with default component config.

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

// ---------- PIN CONFIG ----------
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz

#define VIBRO_PIN          (gpio_num_t) 26
#define BUZZER_PIN         (gpio_num_t) 25
#define BUTTON_PIN         (gpio_num_t) 0    // user acknowledge
#define LED_PIN            (gpio_num_t) 2
#define GSM_TX_PIN         (gpio_num_t) 17
#define GSM_RX_PIN         (gpio_num_t) 16
#define BAT_ADC_CHANNEL    ADC1_CHANNEL_6 // GPIO34 (example)
#define WORN_CAP_PIN       (gpio_num_t) 33   // optional

// ---------- I2C addresses ----------
#define MPU6050_ADDR        0x68
#define MAX30102_ADDR       0x57

// ---------- Parameters (tune) ----------
#define INACTIVITY_WINDOW_MS   (5 * 60 * 1000)
#define MOTION_SAMPLE_MS       200
#define MOTION_RMS_THRESHOLD   0.08f

#define HR_SAMPLING_RATE_HZ    100      // target sample rate for MAX30102 in this simple code
#define HR_CHECK_DURATION_MS   (15 * 1000)
#define HR_LOW_BPM             40
#define HR_HIGH_BPM            140
#define ACK_TIMEOUT_MS         (30 * 1000)

#define BATTERY_LOW_VOLTAGE    3.5f

// ---------- Event queue ----------
typedef enum {
    EVT_MOTION,
    EVT_NO_MOTION,
    EVT_BATT_LOW,
    EVT_BATT_OK,
    EVT_WORN,
    EVT_NOT_WORN,
    EVT_HR_ABNORMAL,
    EVT_HR_OK,
    EVT_EMERGENCY
} event_t;

static QueueHandle_t evt_queue = NULL;

// ---------- I2C helpers ----------
static esp_err_t i2c_master_init(void){
    i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if(ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ---------- MPU6050 minimal init + read ----------
static esp_err_t mpu6050_init(void){
    // wake device: PWR_MGMT_1 = 0x00
    esp_err_t r = i2c_write_reg(MPU6050_ADDR, 0x6B, 0x00);
    if(r != ESP_OK) return r;
    // set accel config to ±2g (0x1C -> 0)
    r = i2c_write_reg(MPU6050_ADDR, 0x1C, 0x00);
    if(r != ESP_OK) return r;
    // set gyro config if needed (0x1B)
    return ESP_OK;
}

// Read raw accel (signed 16-bit big-endian) into floats in g
static bool mpu6050_read_accel(float *ax, float *ay, float *az){
    uint8_t buf[6];
    if(i2c_read_regs(MPU6050_ADDR, 0x3B, buf, 6) != ESP_OK){
        return false;
    }
    int16_t rx = (buf[0] << 8) | buf[1];
    int16_t ry = (buf[2] << 8) | buf[3];
    int16_t rz = (buf[4] << 8) | buf[5];
    // for ±2g, scale = 16384 LSB/g
    const float scale = 16384.0f;
    *ax = ((float)rx) / scale;
    *ay = ((float)ry) / scale;
    *az = ((float)rz) / scale;
    return true;
}

// ---------- MAX30102 init + FIFO read (simple) ----------
static esp_err_t max30102_write(uint8_t reg, uint8_t val){
    return i2c_write_reg(MAX30102_ADDR, reg, val);
}

static esp_err_t max30102_read(uint8_t reg, uint8_t *buf, size_t len){
    return i2c_read_regs(MAX30102_ADDR, reg, buf, len);
}

static esp_err_t max30102_init(void){
    // Reset
    if(max30102_write(0x09, 0x40) != ESP_OK) return ESP_FAIL; // Mode config reset bit
    vTaskDelay(pdMS_TO_TICKS(100));
    // FIFO config: sample averaging 4, fifo rollover disabled (0x08)
    if(max30102_write(0x08, 0x0f) != ESP_OK) return ESP_FAIL;
    // Mode config: HR only (0x03)
    if(max30102_write(0x09, 0x03) != ESP_OK) return ESP_FAIL;
    // SpO2 config: ADC range 4096nA, sample rate 100Hz, LED pulse width 411us (0x27)
    if(max30102_write(0x0A, 0x27) != ESP_OK) return ESP_FAIL;
    // LED1 (red) and LED2 (IR) pulse amplitude: tune (start moderate)
    if(max30102_write(0x0C, 0x24) != ESP_OK) return ESP_FAIL; // LED1 (RED)
    if(max30102_write(0x0D, 0x24) != ESP_OK) return ESP_FAIL; // LED2 (IR)
    return ESP_OK;
}

// Read a single 3-byte sample from FIFO (IR reading) — returns 18-bit value
static bool max30102_read_fifo_once(uint32_t *ir_value){
    // FIFO read pointer and data read
    uint8_t data[6];
    if(max30102_read(0x07, data, 6) != ESP_OK) return false;
    // FIFO bytes: RED(3 bytes) then IR(3 bytes) if using both; depends on configuration.
    // Many dev boards return [MSB ... LSB] per sample; adapt if necessary.
    // We'll assume order: RED3, RED2, RED1, IR3 IR2 IR1 -> and we pick IR
    uint32_t ir = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
    ir &= 0x3FFFF; // 18 bits
    *ir_value = ir;
    return true;
}

// ---------- Simple peak-detection BPM estimator ----------
// Collect IR samples at approx fs Hz for duration_ms, then detect peaks and compute BPM.
// This is intentionally simple: remove DC by subtracting moving average, detect threshold crossings.
// Returns BPM (integer) or -1 if unreliable.
static int compute_bpm_from_ir(uint32_t *samples, size_t n_samples, int fs){
    if(n_samples < (size_t)(fs * 3)) return -1; // need at least ~3 sec
    // 1) convert to float, compute simple moving average (window 0.75s)
    int ma_window = fs * 3 / 4;
    if(ma_window < 1) ma_window = 1;
    float *f = (float*) malloc(sizeof(float) * n_samples);
    if(!f) return -1;
    for(size_t i=0;i<n_samples;i++) f[i] = (float)samples[i];
    // compute moving average (prefix sum for speed)
    double *pref = (double*) malloc(sizeof(double)*(n_samples+1));
    if(!pref){ free(f); return -1; }
    pref[0] = 0;
    for(size_t i=0;i<n_samples;i++) pref[i+1] = pref[i] + f[i];
    float *d = (float*) malloc(sizeof(float)*n_samples); // detrended
    if(!d){ free(f); free(pref); return -1; }
    for(size_t i=0;i<n_samples;i++){
        size_t start = (i >= (size_t)ma_window) ? i - ma_window + 1 : 0;
        size_t len = i - start + 1;
        double mean = (pref[i+1] - pref[start]) / (double)len;
        d[i] = f[i] - (float)mean;
    }
    // 2) detect peaks: simple threshold = stddev * factor
    // compute stddev
    double sum = 0;
    for(size_t i=0;i<n_samples;i++) sum += d[i]*d[i];
    double var = sum / n_samples;
    float std = sqrt(var);
    float threshold = std * 1.0f; // tune: higher reduces false positives
    // find peaks as local maxima above threshold
    int peak_count = 0;
    float last_peak_time = -10000.0f;
    float min_peak_interval = 0.35f; // seconds (max plausible HR ~170)
    float total_interval = 0.0f;
    int intervals = 0;
    for(size_t i=1;i+1<n_samples;i++){
        if(d[i] > threshold && d[i] > d[i-1] && d[i] >= d[i+1]){
            float t = (float)i / fs;
            if(last_peak_time < 0 || (t - last_peak_time) >= min_peak_interval){
                if(last_peak_time >= 0){
                    total_interval += (t - last_peak_time);
                    intervals++;
                }
                last_peak_time = t;
                peak_count++;
            }
        }
    }
    int bpm = -1;
    if(intervals > 0){
        float avg_interval = total_interval / intervals; // seconds per beat
        float hr = 60.0f / avg_interval;
        bpm = (int)roundf(hr);
    }
    free(f); free(pref); free(d);
    // Validate bpm
    if(bpm < 30 || bpm > 220) return -1;
    return bpm;
}

// ---------- Read IR samples from MAX30102 for duration_ms at approx fs (blocking) ----------
static int read_hr_from_max30102(int duration_ms, int fs){
    int n_samples_target = (fs * duration_ms) / 1000;
    uint32_t *samples = (uint32_t*) malloc(sizeof(uint32_t) * n_samples_target);
    if(!samples) return -1;
    int collected = 0;
    int delay_per_sample_ms = 1000 / fs;
    int timeout_ms = duration_ms + 2000;
    uint64_t start = esp_timer_get_time()/1000;
    while(collected < n_samples_target && (int)(esp_timer_get_time()/1000 - start) < timeout_ms){
        uint32_t val;
        if(!max30102_read_fifo_once(&val)){
            // if read fails, wait a bit
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        samples[collected++] = val;
        vTaskDelay(pdMS_TO_TICKS(delay_per_sample_ms));
    }
    int bpm = -1;
    if(collected > 0){
        bpm = compute_bpm_from_ir(samples, collected, fs);
    }
    free(samples);
    return bpm;
}

// ---------- Battery read ----------
static float read_battery_voltage(void){
    int raw = adc1_get_raw(BAT_ADC_CHANNEL);
    float v = ((float)raw / 4095.0f) * 3.3f * 2.0f; // adjust divider
    return v;
}

// ---------- Simple GSM UART helpers ----------
#define GSM_UART_NUM UART_NUM_1
#define GSM_UART_BAUDRATE 115200
#define GSM_UART_BUF_SIZE 2048

static void gsm_uart_setup(){
    const uart_config_t uart_config = {
        .baud_rate = GSM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(GSM_UART_NUM, &uart_config);
    uart_set_pin(GSM_UART_NUM, GSM_TX_PIN, GSM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GSM_UART_NUM, GSM_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void gsm_send_at(const char* cmd){
    char buf[256];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    uart_write_bytes(GSM_UART_NUM, buf, strlen(buf));
    ESP_LOGI(TAG, "GSM AT-> %s", cmd);
}

static void gsm_send_sms(const char* number, const char* text){
    gsm_send_at("AT+CMGF=1");
    vTaskDelay(pdMS_TO_TICKS(500));
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    gsm_send_at(cmd);
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(GSM_UART_NUM, text, strlen(text));
    const char ctrlz = 0x1A;
    uart_write_bytes(GSM_UART_NUM, &ctrlz, 1);
    ESP_LOGI(TAG,"SMS queued");
}

static void gsm_call_number(const char* number){
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ATD%s;", number);
    gsm_send_at(cmd);
    ESP_LOGI(TAG,"Call initiated");
}

// ---------- Utility actuators ----------
static inline void vibro_on(){ gpio_set_level(VIBRO_PIN, 1); }
static inline void vibro_off(){ gpio_set_level(VIBRO_PIN, 0); }
static inline void buzzer_on(){ gpio_set_level(BUZZER_PIN, 1); }
static inline void buzzer_off(){ gpio_set_level(BUZZER_PIN, 0); }

static void alarm_beep(int times, int on_ms, int off_ms){
    for(int i=0;i<times;i++){
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        buzzer_off();
        if(off_ms>0) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

// ---------- Main monitor task (state machine) ----------
static void monitor_task(void *pv){
    uint64_t last_motion_ts = esp_timer_get_time()/1000;
    bool battery_low = false;
    bool currently_worn = false;

    while(1){
        // motion sample
        float ax, ay, az;
        if(mpu6050_read_accel(&ax,&ay,&az)){
            float gx = ax, gy = ay, gz = az - 1.0f;
            float rms = sqrtf(gx*gx + gy*gy + gz*gz);
            if(rms > MOTION_RMS_THRESHOLD){
                last_motion_ts = esp_timer_get_time()/1000;
                // (optional) send EVT_MOTION
            } else {
                uint64_t now = esp_timer_get_time()/1000;
                if(now - last_motion_ts >= INACTIVITY_WINDOW_MS){
                    ESP_LOGI(TAG,"No motion for window and device %s", currently_worn ? "(worn)":"(not worn)");
                    if(currently_worn && !battery_low){
                        // start HR check
                        ESP_LOGI(TAG,"Starting HR read...");
                        int bpm = read_hr_from_max30102(HR_CHECK_DURATION_MS, HR_SAMPLING_RATE_HZ);
                        if(bpm <= 0){
                            ESP_LOGW(TAG,"No HR signal");
                            // vibro + wait ack
                            vibro_on(); vTaskDelay(pdMS_TO_TICKS(3000)); vibro_off();
                            // wait for button press
                            uint64_t start = esp_timer_get_time()/1000;
                            bool ack=false;
                            while(esp_timer_get_time()/1000 - start < ACK_TIMEOUT_MS){
                                if(gpio_get_level(BUTTON_PIN) == 1){ ack=true; break; }
                                vTaskDelay(pdMS_TO_TICKS(200));
                            }
                            if(!ack){
                                // emergency
                                xQueueSend(evt_queue, &(event_t){EVT_EMERGENCY}, 0);
                            }
                        } else {
                            ESP_LOGI(TAG,"BPM measured: %d", bpm);
                            if(bpm < HR_LOW_BPM || bpm > HR_HIGH_BPM){
                                xQueueSend(evt_queue, &(event_t){EVT_HR_ABNORMAL}, 0);
                            } else {
                                xQueueSend(evt_queue, &(event_t){EVT_HR_OK}, 0);
                            }
                        }
                        // reset last_motion_ts to avoid re-triggering immediately
                        last_motion_ts = esp_timer_get_time()/1000;
                    }
                }
            }
        } else {
            ESP_LOGW(TAG,"MPU read fail");
        }

        // battery check
        float vbat = read_battery_voltage();
        if(vbat <= BATTERY_LOW_VOLTAGE && !battery_low){
            battery_low = true;
            xQueueSend(evt_queue, &(event_t){EVT_BATT_LOW}, 0);
        } else if(vbat > BATTERY_LOW_VOLTAGE && battery_low){
            battery_low = false;
            xQueueSend(evt_queue, &(event_t){EVT_BATT_OK}, 0);
        }

        // worn detection (try using IR amplitude via MAX30102 or a capacitive pin)
        // For reliability, prefer reading an IR value and check magnitude:
        uint32_t irtest = 0;
        bool worn = false;
        if(max30102_read_fifo_once(&irtest)){
            // if IR amplitude > small threshold -> likely worn
            if(irtest > 5000) worn = true; // tune threshold
        } else {
            // fallback to capacitive pad GPIO
            if(gpio_get_level(WORN_CAP_PIN) == 1) worn = true;
        }
        if(worn && !currently_worn){
            currently_worn = true;
            xQueueSend(evt_queue, &(event_t){EVT_WORN}, 0);
        } else if(!worn && currently_worn){
            currently_worn = false;
            xQueueSend(evt_queue, &(event_t){EVT_NOT_WORN}, 0);
        }

        // Handle queued events
        event_t ev;
        while(xQueueReceive(evt_queue, &ev, 0) == pdTRUE){
            switch(ev){
                case EVT_HR_ABNORMAL:
                    ESP_LOGW(TAG,"Abnormal HR -> vibrate + wait ack");
                    for(int i=0;i<3;i++){
                        vibro_on(); buzzer_on();
                        vTaskDelay(pdMS_TO_TICKS(500));
                        vibro_off(); buzzer_off();
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }
                    {
                        uint64_t start = esp_timer_get_time()/1000;
                        bool ack=false;
                        while(esp_timer_get_time()/1000 - start < ACK_TIMEOUT_MS){
                            if(gpio_get_level(BUTTON_PIN) == 1){ ack=true; break; }
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                        if(!ack) xQueueSend(evt_queue, &(event_t){EVT_EMERGENCY}, 0);
                    }
                    break;
                case EVT_EMERGENCY:
                    ESP_LOGE(TAG,"EMERGENCY triggered!");
                    for(int i=0;i<6;i++){
                        buzzer_on(); vibro_on();
                        vTaskDelay(pdMS_TO_TICKS(700));
                        buzzer_off(); vibro_off();
                        vTaskDelay(pdMS_TO_TICKS(300));
                    }
                    gsm_send_sms("+1234567890", "Emergency: bracelet detected issue, no response.");
                    gsm_call_number("+1234567890");
                    break;
                case EVT_BATT_LOW:
                    ESP_LOGW(TAG,"Battery low!");
                    alarm_beep(2, 300, 200);
                    gsm_send_sms("+1234567890", "Bracelet battery low.");
                    break;
                case EVT_NOT_WORN:
                    ESP_LOGI(TAG,"Not worn reminder");
                    alarm_beep(1, 400, 0);
                    break;
                case EVT_WORN:
                    ESP_LOGI(TAG,"Now worn");
                    break;
                default:
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MOTION_SAMPLE_MS));
    }
}

// ---------- GSM task (simple keepalive) ----------
static void gsm_task(void *pv){
    gsm_uart_setup();
    vTaskDelay(pdMS_TO_TICKS(1000));
    gsm_send_at("AT");
    vTaskDelay(pdMS_TO_TICKS(500));
    gsm_send_at("ATE0");
    vTaskDelay(pdMS_TO_TICKS(200));
    gsm_send_at("AT+CMGF=1");
    while(1) vTaskDelay(pdMS_TO_TICKS(5000));
}

// ---------- app_main ----------
void app_main(void){
    ESP_LOGI(TAG,"Starting MPU6050+MAX30102 bracelet app");

    // init i2c
    if(i2c_master_init() != ESP_OK){
        ESP_LOGE(TAG,"I2C init failed");
        return;
    }
    // init sensors
    if(mpu6050_init() != ESP_OK) ESP_LOGW(TAG,"MPU init failed (check wiring)");
    if(max30102_init() != ESP_OK) ESP_LOGW(TAG,"MAX30102 init failed (check wiring/power)");

    // init gpio
    gpio_pad_select_gpio(VIBRO_PIN); gpio_set_direction(VIBRO_PIN, GPIO_MODE_OUTPUT); vibro_off();
    gpio_pad_select_gpio(BUZZER_PIN); gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT); buzzer_off();
    gpio_pad_select_gpio(BUTTON_PIN); gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLDOWN_ONLY);
    gpio_pad_select_gpio(WORN_CAP_PIN); gpio_set_direction(WORN_CAP_PIN, GPIO_MODE_INPUT); gpio_set_pull_mode(WORN_CAP_PIN, GPIO_PULLDOWN_ONLY);
    gpio_pad_select_gpio(LED_PIN); gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); gpio_set_level(LED_PIN, 1);

    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BAT_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // create event queue
    evt_queue = xQueueCreate(10, sizeof(event_t));
    if(!evt_queue){
        ESP_LOGE(TAG,"Queue create failed");
        return;
    }

    // tasks
    xTaskCreate(monitor_task, "monitor_task", 12*1024, NULL, 5, NULL);
    xTaskCreate(gsm_task, "gsm_task", 4*1024, NULL, 4, NULL);

    ESP_LOGI(TAG,"Tasks started");
}
