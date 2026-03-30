#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include "mqttx.h"
#include "../config/config.h"
#include "../config/state.h"

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ✅ Mutex แทน Queue
// sonarTask เรียก mqttPublish() โดยตรงได้เลย
// Mutex การันตีว่ามีแค่ task เดียวที่แตะ client ในเวลาเดียวกัน
SemaphoreHandle_t mqttMutex = nullptr;

static bool lastWifiConnected = false;
static bool lastMqttConnected = false;

// ─────────────────────────────────────────────────────────────────
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

    char updatedFields[128] = "";
    bool first = true;

    auto appendField = [&](const char *name)
    {
        if (!first)
            strncat(updatedFields, ",", sizeof(updatedFields) - strlen(updatedFields) - 1);
        strncat(updatedFields, "\"", sizeof(updatedFields) - strlen(updatedFields) - 1);
        strncat(updatedFields, name, sizeof(updatedFields) - strlen(updatedFields) - 1);
        strncat(updatedFields, "\"", sizeof(updatedFields) - strlen(updatedFields) - 1);
        first = false;
    };

    if (!doc["isSystemStart"].isNull())
    {
        isSystemStart = doc["isSystemStart"].as<bool>();
        Serial.printf("🔧 isSystemStart: %s\n", isSystemStart ? "true" : "false");
        appendField("isSystemStart");
    }
    if (!doc["isAlert"].isNull())
    {
        isAlert = doc["isAlert"].as<bool>();
        Serial.printf("🔔 isAlert: %s\n", isAlert ? "true" : "false");
        appendField("isAlert");
    }
    if (!doc["mode"].isNull())
    {
        strncpy((char *)mode, doc["mode"], sizeof(mode) - 1);
        Serial.printf("🔄 mode: %s\n", (char *)mode);
        appendField("mode");
    }
    if (!doc["pos"].isNull())
    {
        pos = doc["pos"].as<int>();
        Serial.printf("📐 pos: %d\n", pos);
        appendField("pos");
    }
    if (!doc["minRotate"].isNull())
    {
        minRotate = doc["minRotate"].as<int>();
        Serial.printf("🔧 minRotate: %d\n", minRotate);
        appendField("minRotate");
    }
    if (!doc["maxRotate"].isNull())
    {
        maxRotate = doc["maxRotate"].as<int>();
        Serial.printf("🔧 maxRotate: %d\n", maxRotate);
        appendField("maxRotate");
    }
    if (!doc["maxRange"].isNull())
    {
        maxRange = doc["maxRange"].as<int>();
        if (RedZone > maxRange)
            RedZone = maxRange;
        Serial.printf("🔧 maxRange: %d\n", maxRange);
        appendField("maxRange");
    }
    if (!doc["RedZone"].isNull())
    {
        RedZone = doc["RedZone"].as<int>();
        if (RedZone > maxRange)
            RedZone = maxRange;
        Serial.printf("🔧 RedZone: %d\n", RedZone);
        appendField("RedZone");
    }

    if (!first)
    {
        char ack[256];
        snprintf(ack, sizeof(ack),
                 "{\"status\":\"updated\",\"fields\":[%s]}", updatedFields);
        // callback ถูกเรียกจาก client.loop() ใน mqttTask เอง
        // ดังนั้น Mutex ถือครองอยู่แล้ว — publish โดยตรงได้เลย
        client.publish("BeanNian/esp32/status", ack, false);
        Serial.printf("✅ ACK → %s\n", ack);
    }
}

// ─────────────────────────────────────────────────────────────────
static void publishStatus(const char *msg)
{
    client.publish("BeanNian/esp32/status", msg, true);
    Serial.printf("📡 status → %s\n", msg);
}

// ─────────────────────────────────────────────────────────────────
static void publishOfflineAndDisconnect()
{
    if (!client.connected())
        return;

    publishStatus("{\"status\":\"offline\"}");

    unsigned long t = millis();
    while (millis() - t < 100)
    {
        client.loop();
        esp_task_wdt_reset();
        taskYIELD();
    }

    client.disconnect();
    Serial.println("🔌 Disconnected");
}

// ─────────────────────────────────────────────────────────────────
bool reconnect()
{
    Serial.println("🔄 Connecting to MQTT...");

    // ถอด WDT ชั่วคราว — TLS handshake blocking ใช้เวลานาน
    esp_task_wdt_delete(NULL);

    bool ok = client.connect(
        "ESP32_BeanNian_001",
        mqtt_user,
        mqtt_pass,
        "BeanNian/esp32/status",
        1,
        true,
        "{\"status\":\"offline\"}");

    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();

    if (ok)
    {
        Serial.println("✅ MQTT connected!");
        publishStatus("{\"status\":\"online\"}");
        client.subscribe("BeanNian/esp32/state", 1);
        Serial.println("📥 Subscribed — waiting for retained state...");

        // รอ 500ms เพื่อรับ retained message ก่อนปล่อยให้ sonarTask วิ่ง
        unsigned long t = millis();
        while (millis() - t < 500)
        {
            client.loop();
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        return true;
    }

    Serial.printf("❌ MQTT failed, rc=%d\n", client.state());
    return false;
}

// ─────────────────────────────────────────────────────────────────
void mqttTask(void *pvParameters)
{
    // ✅ สร้าง Mutex ที่นี่ — mqttTask เป็น owner ของ client
    mqttMutex = xSemaphoreCreateMutex();

    esp_task_wdt_add(NULL);

    while (WiFi.status() != WL_CONNECTED)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    espClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setKeepAlive(15);
    client.setSocketTimeout(10);
    client.setCallback(callback);

    unsigned long lastReconnect = 0;

    for (;;)
    {
        esp_task_wdt_reset();

        bool wifiNow = (WiFi.status() == WL_CONNECTED);

        if (lastWifiConnected && !wifiNow)
        {
            Serial.println("⚠️  WiFi lost");
            if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                publishOfflineAndDisconnect();
                xSemaphoreGive(mqttMutex);
            }
            lastMqttConnected = false;
        }
        lastWifiConnected = wifiNow;

        if (!wifiNow)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!client.connected())
        {
            if (millis() - lastReconnect > 3000)
            {
                lastReconnect = millis();
                // reconnect ไม่ต้องล็อก Mutex เพราะ client ยังไม่ connected
                lastMqttConnected = reconnect();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ✅ lock Mutex ก่อน client.loop()
        // mqttPublish() จาก sonarTask จะรอ Mutex นี้ก่อนส่ง
        if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            client.loop();
            xSemaphoreGive(mqttMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─────────────────────────────────────────────────────────────────
// sonarTask เรียกตรงนี้ได้เลย — Mutex ป้องกัน race condition
void mqttPublish(const char *topic, const char *message)
{
    if (mqttMutex == nullptr || !client.connected())
        return;

    // รอ Mutex ไม่เกิน 100ms — ถ้า timeout ให้ทิ้งข้อมูลนี้ไป
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        client.publish(topic, message, true);
        xSemaphoreGive(mqttMutex);
    }
    else
    {
        Serial.println("⚠️  mqttPublish timeout — message dropped");
    }
}

// ─────────────────────────────────────────────────────────────────
void mqttGracefulDisconnect()
{
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        publishOfflineAndDisconnect();
        xSemaphoreGive(mqttMutex);
    }
}