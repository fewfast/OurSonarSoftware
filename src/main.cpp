#include <Arduino.h>
#include <WiFi.h>
#include "esp_task_wdt.h"
#include "./config/config.h"
#include "./network/wifi.h"
#include "./network/mqttx.h"
#include "./task/sonar.h"

#define WDT_TIMEOUT 60

void setup()
{
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_task_wdt_init(WDT_TIMEOUT, true);

    xTaskCreatePinnedToCore(
        connectWiFi, "wifiTask",
        4096, NULL, 2, NULL, 0); // Core 0

    xTaskCreatePinnedToCore(
        mqttTask, "mqttTask",
        8192, NULL, 2, NULL, 0); // Core 0

    xTaskCreatePinnedToCore(
        sonarTask, "sonarTask",
        8192, NULL, 2, NULL, 1); // Core 1
}

void loop()
{
    vTaskDelete(NULL);
}