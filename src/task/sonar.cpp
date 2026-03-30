#include <Arduino.h>
#include <Servo.h>
#include "sonar.h"
#include "esp_task_wdt.h"
#include "../network/mqttx.h"
#include "../config/state.h"

#define SERVO_PIN 13
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DETECT_MIN_CM 2
#define MAX_OBJECTS 20
#define TRACK_SCAN_RANGE 10
#define NULL_TOLERANCE 3

// ✅ FIX: เพิ่ม timeout รอ MQTT ready ก่อน sonarTask วิ่ง
// ป้องกัน retained message มาเปลี่ยน isSystemStart หลัง servo เริ่มแล้ว
#define MQTT_READY_TIMEOUT_MS 3000

volatile bool alertEnabled = true;
volatile bool servoEnabled = true;

Servo radarServo;

struct DetectedObject
{
    int angle;
    float distance;
};

float getDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0)
        return -1;
    return duration * 0.034 / 2;
}

const char *getType(float dist)
{
    if (isAlert && dist < RedZone)
        return "RedZone";
    return "Notice";
}

void reportRoam(DetectedObject *objects, int count)
{
    if (count == 0)
        return;

    char msg[1024];
    int offset = 0;

    offset += snprintf(msg + offset, sizeof(msg) - offset,
                       "{\"mode\":\"ROAM\",\"count\":%d,\"objects\":[", count);

    for (int i = 0; i < count; i++)
    {
        offset += snprintf(msg + offset, sizeof(msg) - offset,
                           "%s{\"type\":\"%s\",\"angle\":%d,\"distance\":%.1f}",
                           i > 0 ? "," : "",
                           getType(objects[i].distance),
                           objects[i].angle,
                           objects[i].distance);
    }

    snprintf(msg + offset, sizeof(msg) - offset, "]}");
    mqttPublish("BeanNian/esp32/report", msg);
    Serial.printf("📊 ROAM report: %s\n", msg);
}

int scanRange(int scanMin, int scanMax)
{
    int bestAngle = -1;
    float bestDist = 9999;

    scanMin = constrain(scanMin, 0, 180);
    scanMax = constrain(scanMax, 0, 180);

    for (int a = scanMin; a <= scanMax; a += 4)
    {
        radarServo.write(a);
        vTaskDelay(pdMS_TO_TICKS(20));

        float d = getDistance();
        if (d > DETECT_MIN_CM && d < maxRange && d < bestDist)
        {
            bestDist = d;
            bestAngle = a;
        }
    }

    return bestAngle;
}

int scanClosest(int centerAngle)
{
    return scanRange(
        centerAngle - TRACK_SCAN_RANGE,
        centerAngle + TRACK_SCAN_RANGE);
}

void sonarTask(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    radarServo.attach(SERVO_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));
    radarServo.write(90);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ✅ FIX: รอให้ mqttMutex พร้อม (แปลว่า mqttTask init เสร็จแล้ว)
    // ช่วงนี้ retained message จะถูก callback แล้ว isSystemStart จะถูกเซ็ตถูกต้อง
    // ป้องกัน sonarTask วิ่งก่อน retained message มา แล้วติด loop ที่ isSystemStart
    Serial.println("⏳ sonarTask: waiting for MQTT ready...");
    unsigned long waitStart = millis();
    while (mqttMutex == nullptr && millis() - waitStart < MQTT_READY_TIMEOUT_MS)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ✅ FIX: รอเพิ่มอีก 600ms ให้ retained message วิ่งผ่าน callback ก่อน
    // mqttTask loop 500ms รับ retained message — รอให้มันจบก่อน
    for (int i = 0; i < 6; i++)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.printf("✅ sonarTask: ready | isSystemStart=%d | mode=%s\n",
                  (int)isSystemStart, (char *)mode);

    // ✅ FIX: ถ้า mode ว่างหรือไม่รู้จัก ให้ default เป็น ROAM
    if (strlen((const char *)mode) == 0 ||
        (strcmp((const char *)mode, "ROAM") != 0 &&
         strcmp((const char *)mode, "TRACK") != 0 &&
         strcmp((const char *)mode, "STATIC") != 0))
    {
        strncpy((char *)mode, "ROAM", sizeof(mode) - 1);
        Serial.println("⚠️  mode unknown — defaulting to ROAM");
    }

    int angle = minRotate;
    int direction = 1;
    bool tracking = false;
    int lockedAngle = 0;
    float lockedDist = 9999;

    DetectedObject objects[MAX_OBJECTS];
    int objectCount = 0;
    bool inGroup = false;
    int groupMinAngle = 0;
    float groupMinDist = 9999;
    int nullCount = 0;

    float prev = 0;
    char lastMode[16] = "";

    for (;;)
    {
        esp_task_wdt_reset();

        // ── ถ้าระบบปิด จอดนิ่งที่ 90° ──────────────────────
        if (!isSystemStart)
        {
            radarServo.write(90);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── reset smoothing เมื่อ mode เปลี่ยน ──────────────
        if (strcmp((const char *)mode, lastMode) != 0)
        {
            prev = 0;
            angle = minRotate;
            direction = 1;
            tracking = false; // ✅ FIX: reset tracking state ด้วยเมื่อ mode เปลี่ยน
            objectCount = 0;  // ✅ FIX: reset object buffer ด้วย
            inGroup = false;
            nullCount = 0;
            strncpy(lastMode, (const char *)mode, sizeof(lastMode) - 1);
            Serial.printf("🔄 Mode changed → %s\n", lastMode);
        }

        // ════════════════════════════════════════════════════
        // STATIC mode
        // ════════════════════════════════════════════════════
        if (strcmp((const char *)mode, "STATIC") == 0)
        {
            int target = constrain(pos, 0, 180);
            radarServo.write(target);

            float dist = getDistance();
            bool inRange = (dist > DETECT_MIN_CM && dist < maxRange);

            if (inRange)
            {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "{\"mode\":\"STATIC\",\"type\":\"%s\",\"detected\":true,\"distance\":%.1f}",
                         getType(dist), dist);
                mqttPublish("BeanNian/esp32/report", msg);
                Serial.printf("📡 STATIC [%d°] %s\n", target, msg);
            }
            else
            {
                Serial.printf("📡 STATIC [%d°] nothing\n", target);
            }

            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // ── Servo move ──────────────────────────────────────
        radarServo.write(angle);
        vTaskDelay(pdMS_TO_TICKS(30));

        // ── อ่านระยะ + smoothing ────────────────────────────
        float dist = getDistance();

        if (dist > DETECT_MIN_CM && dist < maxRange)
        {
            dist = dist * 0.7f + prev * 0.3f;
            prev = dist;
        }
        else
        {
            prev = 0;
        }

        bool inRange = (dist > DETECT_MIN_CM && dist < maxRange);

        // ════════════════════════════════════════════════════
        // TRACK mode
        // ════════════════════════════════════════════════════
        if (strcmp((const char *)mode, "TRACK") == 0)
        {
            if (!tracking)
            {
                if (inRange)
                {
                    lockedAngle = constrain(angle, 0, 180);
                    lockedDist = dist;
                    tracking = true;
                    Serial.printf("🎯 TRACK locked at %d° %.1f cm\n", lockedAngle, lockedDist);
                }
            }
            else
            {
                radarServo.write(lockedAngle);
                vTaskDelay(pdMS_TO_TICKS(20));
                float currentDist = getDistance();

                bool stillThere = (currentDist > DETECT_MIN_CM && currentDist < maxRange);
                bool moved = stillThere && fabsf(currentDist - lockedDist) > 20.0f;

                if (!stillThere || moved)
                {
                    int newAngle = scanClosest(lockedAngle);
                    if (newAngle != -1)
                    {
                        radarServo.write(newAngle);
                        vTaskDelay(pdMS_TO_TICKS(20));
                        lockedAngle = newAngle;
                        lockedDist = getDistance();
                        Serial.printf("🎯 TRACK re-locked at %d° %.1f cm\n", lockedAngle, lockedDist);
                    }
                    else
                    {
                        int fullAngle = scanRange(minRotate, maxRotate);
                        if (fullAngle != -1)
                        {
                            radarServo.write(fullAngle);
                            vTaskDelay(pdMS_TO_TICKS(20));
                            lockedAngle = fullAngle;
                            lockedDist = getDistance();
                            Serial.printf("🎯 TRACK found at %d° %.1f cm\n", lockedAngle, lockedDist);
                        }
                        else
                        {
                            tracking = false;
                            Serial.println("❌ TRACK lost target");
                            vTaskDelay(pdMS_TO_TICKS(30));
                            continue;
                        }
                    }
                }
                else
                {
                    lockedDist = currentDist;
                }

                angle = lockedAngle;
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "{\"mode\":\"TRACK\",\"type\":\"%s\",\"angle\":%d,\"distance\":%.1f}",
                         getType(lockedDist), lockedAngle, lockedDist);
                mqttPublish("BeanNian/esp32/report", msg);
                Serial.printf("📡 TRACK [%d°] %.1f cm\n", lockedAngle, lockedDist);

                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }
        else
        {
            tracking = false;
        }

        // ════════════════════════════════════════════════════
        // ROAM mode — grouping
        // ════════════════════════════════════════════════════
        if (strcmp((const char *)mode, "ROAM") == 0)
        {
            if (inRange)
            {
                nullCount = 0;
                if (!inGroup)
                {
                    inGroup = true;
                    groupMinDist = dist;
                    groupMinAngle = angle;
                }
                else
                {
                    if (fabsf(dist - groupMinDist) <= 20.0f)
                    {
                        if (dist < groupMinDist)
                        {
                            groupMinDist = dist;
                            groupMinAngle = angle;
                        }
                    }
                    else
                    {
                        if (objectCount < MAX_OBJECTS)
                        {
                            objects[objectCount].angle = groupMinAngle;
                            objects[objectCount].distance = groupMinDist;
                            objectCount++;
                        }
                        groupMinDist = dist;
                        groupMinAngle = angle;
                    }
                }
            }
            else
            {
                if (inGroup)
                {
                    nullCount++;
                    if (nullCount >= NULL_TOLERANCE)
                    {
                        if (objectCount < MAX_OBJECTS)
                        {
                            objects[objectCount].angle = groupMinAngle;
                            objects[objectCount].distance = groupMinDist;
                            objectCount++;
                        }
                        inGroup = false;
                        nullCount = 0;
                    }
                }
            }
        }

        // ── เลื่อน angle ────────────────────────────────────
        angle += direction * 2;

        if (angle >= maxRotate)
        {
            angle = maxRotate;
            direction = -1;

            if (strcmp((const char *)mode, "ROAM") == 0)
            {
                if (inGroup && objectCount < MAX_OBJECTS)
                {
                    objects[objectCount].angle = groupMinAngle;
                    objects[objectCount].distance = groupMinDist;
                    objectCount++;
                    inGroup = false;
                }
                reportRoam(objects, objectCount);
                objectCount = 0;
                nullCount = 0;
            }
        }
        else if (angle <= minRotate)
        {
            angle = minRotate;
            direction = 1;

            if (strcmp((const char *)mode, "ROAM") == 0)
            {
                if (inGroup && objectCount < MAX_OBJECTS)
                {
                    objects[objectCount].angle = groupMinAngle;
                    objects[objectCount].distance = groupMinDist;
                    objectCount++;
                    inGroup = false;
                }
                reportRoam(objects, objectCount);
                objectCount = 0;
                nullCount = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}