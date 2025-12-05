#ifndef CONSTANTS_H
#define CONSTANTS_H

//--------PINS-------------

#define SDA_PIN     21
#define SCL_PIN     22
#define VIBRO_PIN   23
#define BUZZER_PIN  18
#define BUTTON_PIN  GPIO_NUM_33
#define LED_PIN     2
#define TX_PIN      17
#define RX_PIN      16
#define BATTERY_PIN 4

//--------TIME CONSTANTS-------------

//microseconds!!!
const int SENSOR_CHECK_PERIOD = 40*1000000;
const int BATTERY_CHECK_PERIOD = 300000000;
const int CONNECTION_CHECK_PERIOD = 300000000;

//--------WIFI CONSTANTS-------------

const String telegramBotToken = "8242846318:AAHPON-9mM39k9YYO01KKtm2JD-_rEdMl5c";
const String telegramChatID_IVAN = "541565068";
const String telegramChatID_VADIM = "1746293662";

#endif
