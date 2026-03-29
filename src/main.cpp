#include <Arduino.h>
#include <WiFi.h>
#include "./config/config.h"
#include "./network/wifi.h"
#include "./network/mqttx.h"
#include "./task/sonar.h"

void setup()
{
    Serial.begin(115200);

    // WiFi ก่อนเลย priority สูงสุด
    xTaskCreatePinnedToCore(
        connectWiFi,
        "wifiTask",
        4096,
        NULL,
        3, // ✅ priority สูงสุด connect ก่อน
        NULL,
        0);

    // MQTT รอ WiFi เองอยู่แล้วใน mqttTask
    xTaskCreatePinnedToCore(
        mqttTask,
        "mqttTask",
        8192, // ✅ เพิ่ม stack เผื่อ TLS
        NULL,
        2,
        NULL,
        0); // core 0

    // Sonar ไป core 1 ไม่แย่งกับ WiFi/MQTT
    xTaskCreatePinnedToCore(
        sonarTask,
        "sonarTask",
        8192, // ✅ เพิ่ม stack เผื่อ JSON buffer + DetectedObject array
        NULL,
        1,
        NULL,
        1); // core 1
}

void loop()
{
    vTaskDelete(NULL); // ไม่ใช้ loop ของ Arduino
}