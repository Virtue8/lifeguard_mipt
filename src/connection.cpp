#include "Arduino.h"
#include <WiFi.h>
#include <HTTPClient.h>

#include "constants.h"
#include "connection.h"

const char* ssid = "Zandik";
const char* password = "fuckkk123";

bool connectToWiFi()
{
    int start_time = millis();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start_time > 10000)
        {
            Serial.println('\n');
            Serial.println("Failed to connect to WIFI");
            return false;
        }
        delay(500);
        Serial.print('.');
    }
    Serial.println('\n');
    Serial.println("Connection established");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());
    return true;
}

void send_tg_message(String message)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;

        String url = "https://api.telegram.org/bot" + telegramBotToken + "/sendMessage";
        String payload = "chat_id=" + telegramChatID_VAN +
                        "&text=" + message +
                        "&parse_mode=HTML";

        http.begin(url);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        int httpCode = http.POST(payload);

        if (httpCode == 200)
            Serial.println("Alert sent to Telegram");
        else
            Serial.println("Failed to send alert: " + String(httpCode));

        http.end();
    }
}
