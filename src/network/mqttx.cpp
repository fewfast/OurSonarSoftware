#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include "mqttx.h"
#include "../config/config.h"
#include "../config/state.h"

WiFiClientSecure espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length)
{
    char msg[256];

    if (length >= sizeof(msg))
        return;

    memcpy(msg, payload, length);
    msg[length] = '\0';

    if (strcmp(topic, "BeanNian/esp32/state") != 0)
        return;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, msg);

    if (err)
    {
        Serial.printf("JSON error: %s\n", err.c_str());
        return;
    }

    if (!doc["isSystemStart"].isNull())
    {
        isSystemStart = doc["isSystemStart"].as<bool>();
        Serial.printf("🔧 isSystemStart: %s\n", isSystemStart ? "true" : "false");
    }

    if (!doc["isAlert"].isNull())
    {
        isAlert = doc["isAlert"].as<bool>();
        Serial.printf("🔔 isAlert: %s\n", isAlert ? "true" : "false");
    }

    if (!doc["mode"].isNull())
    {
        strncpy((char *)mode, doc["mode"], sizeof(mode) - 1);
        Serial.printf("🔄 mode: %s\n", (char *)mode);
    }

    if (!doc["pos"].isNull())
    {
        pos = doc["pos"].as<int>();
        Serial.printf("📐 pos: %d\n", pos);
    }

    if (!doc["minRotate"].isNull())
    {
        minRotate = doc["minRotate"].as<int>();
        Serial.printf("🔧 minRotate: %d\n", minRotate);
    }

    if (!doc["maxRotate"].isNull())
    {
        maxRotate = doc["maxRotate"].as<int>();
        Serial.printf("🔧 maxRotate: %d\n", maxRotate);
    }

    if (!doc["maxRange"].isNull())
    {
        maxRange = doc["maxRange"].as<int>();
        Serial.printf("🔧 maxRange: %d\n", maxRange);
    }

    if (!doc["RedZone"].isNull())
    {
        RedZone = doc["RedZone"].as<int>();
        Serial.printf("🔧 RedZone: %d\n", RedZone);
    }
}

bool reconnect()
{
    esp_task_wdt_reset();

    bool ok = client.connect(
        "ESP32_BeanNian_001",
        mqtt_user,
        mqtt_pass,
        "BeanNian/esp32/status",
        0,
        true,
        "ESP32-Disconnected");

    esp_task_wdt_reset();

    if (ok)
    {
        Serial.println("MQTT connected!");
        client.publish("BeanNian/esp32/status", "ESP32-Connected", true);
        client.subscribe("BeanNian/esp32/state"); // ✅ subscribe ถูก topic
        return true;
    }
    else
    {
        Serial.print("failed, rc=");
        Serial.println(client.state());
        return false;
    }
}

void mqttTask(void *pvParameters)
{
    while (WiFi.status() != WL_CONNECTED)
        vTaskDelay(pdMS_TO_TICKS(500));

    espClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    unsigned long lastReconnect = 0;

    for (;;)
    {
        if (!client.connected())
        {
            if (millis() - lastReconnect > 3000)
            {
                lastReconnect = millis();
                reconnect();
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        unsigned long start = millis();
        while (millis() - start < 10)
        {
            client.loop();
            taskYIELD();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void mqttPublish(const char *topic, const char *message)
{
    if (client.connected())
        client.publish(topic, message, true);
    else
        Serial.println("❌ mqttPublish failed: not connected"); // ✅ เพิ่ม
}