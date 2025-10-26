#include "../inc/lifeguard.h"

void main ()
{
    ESP_LOGI (TAG, "Starting MPU6050+MAX30102 bracelet app");

    // init i2c
    if (i2c_master_init() != ESP_OK)
    {
        ESP_LOGE(TAG,"I2C init failed");
        return;
    }
    // init sensors
    if (mpu6050_init () != ESP_OK) ESP_LOGW (TAG,"MPU init failed (check wiring)");
    if (max30102_init () != ESP_OK) ESP_LOGW (TAG,"MAX30102 init failed (check wiring/power)");

    // init gpio
    gpio_pad_select_gpio (VIBRO_PIN); gpio_set_direction(VIBRO_PIN, GPIO_MODE_OUTPUT); vibro_off();
    gpio_pad_select_gpio (BUZZER_PIN); gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT); buzzer_off();
    gpio_pad_select_gpio (BUTTON_PIN); gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLDOWN_ONLY);
    gpio_pad_select_gpio (WORN_CAP_PIN); gpio_set_direction(WORN_CAP_PIN, GPIO_MODE_INPUT); gpio_set_pull_mode(WORN_CAP_PIN, GPIO_PULLDOWN_ONLY);
    gpio_pad_select_gpio (LED_PIN); gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); gpio_set_level(LED_PIN, 1);

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
    xTaskCreate (monitor_task, "monitor_task", 12*1024, NULL, 5, NULL);
    xTaskCreate (gsm_task, "gsm_task", 4*1024, NULL, 4, NULL);

    ESP_LOGI(TAG,"Tasks started");
}
