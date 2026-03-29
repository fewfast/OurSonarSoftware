#include <WiFi.h>
#include "wifi.h"
#include "../config/config.h"

#define WIFI_RETRY_DELAY 500
#define WIFI_MAX_RETRY 40

void connectWiFi(void *pvParameters)
{
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("ESP32-BeanNian");
    WiFi.setAutoReconnect(true);

    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY));
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    static bool wasConnected = true;

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            if (wasConnected)
            {
                Serial.println("\nWiFi lost! Reconnecting...");
                wasConnected = false;
            }

            WiFi.reconnect();

            while (WiFi.status() != WL_CONNECTED)
            {
                vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY));
                Serial.print(".");
            }

            Serial.println("\nReconnected!");
            Serial.print("New IP: ");
            Serial.println(WiFi.localIP());
            Serial.print("RSSI: ");
            Serial.println(WiFi.RSSI());

            wasConnected = true;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}