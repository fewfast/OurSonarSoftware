#include <WiFi.h>
#include "wifi.h"
#include "../config/config.h"
#include "esp_task_wdt.h" // ✅ เพิ่ม

#define WIFI_RETRY_DELAY 500
// WIFI_MAX_RETRY ลบออก — ไม่ได้ใช้จริง, while วนจน connect ได้จริงๆ

void connectWiFi(void *pvParameters)
{
    esp_task_wdt_add(NULL); // ✅ add WDT ครั้งเดียวตอนเริ่ม และไม่ลบออกเด็ดขาด
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("ESP32-BeanNian");
    WiFi.setAutoReconnect(true);

    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");

    // ✅ รอ connect ครั้งแรก พร้อม reset WDT ทุก iteration
    while (WiFi.status() != WL_CONNECTED)
    {
        esp_task_wdt_reset(); // ✅ FIX: ป้องกัน WDT timeout ขณะรอ connect
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_DELAY));
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());

    bool wasConnected = true; // ✅ ย้ายออกจาก static — ไม่จำเป็นต้องเป็น static

    for (;;)
    {
        esp_task_wdt_reset(); // ✅ FIX: reset WDT ต้องอยู่นอก if เสมอ (เดิมอยู่ผิดที่)

        if (WiFi.status() != WL_CONNECTED)
        {
            if (wasConnected)
            {
                Serial.println("\nWiFi lost! Reconnecting...");
                wasConnected = false;
            }

            WiFi.reconnect();

            // ✅ FIX: เพิ่ม WDT reset ใน inner while ป้องกัน reboot loop
            while (WiFi.status() != WL_CONNECTED)
            {
                esp_task_wdt_reset();
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

        // ✅ FIX: ลบ esp_task_wdt_delete(NULL) ออก — ไม่ควรลบ task จาก WDT เลย
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
