#include <Arduino.h>
#include <Servo.h>
#include "sonar.h"
#include "../network/mqttx.h"
#include "../config/state.h"

#define SERVO_PIN 13
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DETECT_MIN_CM 2
#define MAX_OBJECTS 20
#define TRACK_SCAN_RANGE 10
#define NULL_TOLERANCE 3

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

// ── scan ระหว่าง scanMin-scanMax หามุมที่ใกล้สุด ────────────
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

    return bestAngle; // -1 ถ้าไม่เจอ
}

// ── scan ±TRACK_SCAN_RANGE° จาก center ────────────────────
int scanClosest(int centerAngle)
{
    return scanRange(
        centerAngle - TRACK_SCAN_RANGE,
        centerAngle + TRACK_SCAN_RANGE);
}

void sonarTask(void *pvParameters)
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    radarServo.attach(SERVO_PIN);
    radarServo.write(minRotate);
    vTaskDelay(pdMS_TO_TICKS(1000));

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

    for (;;)
    {
        // ── ถ้าระบบปิด ─────────────────────────────────────
        if (!isSystemStart)
        {
            if (radarServo.attached())
                radarServo.detach();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ════════════════════════════════════════════════════
        // STATIC mode
        // ════════════════════════════════════════════════════
        if (strcmp((const char *)mode, "STATIC") == 0)
        {
            int target = constrain(pos, 0, 180);
            if (!radarServo.attached())
                radarServo.attach(SERVO_PIN);
            radarServo.write(target);

            float dist = getDistance();
            bool inRange = (dist > DETECT_MIN_CM && dist < maxRange);

            char msg[80];
            if (inRange)
                snprintf(msg, sizeof(msg),
                         "{\"mode\":\"STATIC\",\"type\":\"%s\",\"detected\":true,\"distance\":%.1f}",
                         getType(dist), dist);
            else
                snprintf(msg, sizeof(msg),
                         "{\"mode\":\"STATIC\",\"type\":\"-\",\"detected\":false}");

            mqttPublish("BeanNian/esp32/report", msg);
            Serial.printf("📡 STATIC [%d°] %s\n", target, msg);

            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // ── Servo move ──────────────────────────────────────
        if (!radarServo.attached())
            radarServo.attach(SERVO_PIN);
        radarServo.write(angle);
        vTaskDelay(pdMS_TO_TICKS(30));

        // ── อ่านระยะ + smoothing ────────────────────────────
        float dist = getDistance();

        static float prev = 0;
        if (dist > DETECT_MIN_CM && dist < maxRange)
        {
            dist = dist * 0.7f + prev * 0.3f;
            prev = dist;
        }

        bool inRange = (dist > DETECT_MIN_CM && dist < maxRange);

        // ════════════════════════════════════════════════════
        // TRACK mode
        // ════════════════════════════════════════════════════
        if (strcmp((const char *)mode, "TRACK") == 0)
        {
            if (!tracking)
            {
                // ยังไม่เจอของ — ROAM หาของก่อน
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
                // กำลัง lock อยู่ — อ่านระยะที่มุม lock
                radarServo.write(lockedAngle);
                vTaskDelay(pdMS_TO_TICKS(20));
                float currentDist = getDistance();

                bool stillThere = (currentDist > DETECT_MIN_CM && currentDist < maxRange);
                bool moved = stillThere && fabsf(currentDist - lockedDist) > 20.0f;

                if (!stillThere || moved)
                {
                    // scan ±10° ก่อน
                    Serial.printf("🔍 TRACK scanning ±%d° around %d°\n", TRACK_SCAN_RANGE, lockedAngle);
                    int newAngle = scanClosest(lockedAngle);

                    if (newAngle != -1)
                    {
                        // เจอใน ±10°
                        radarServo.write(newAngle);
                        vTaskDelay(pdMS_TO_TICKS(20));
                        float newDist = getDistance();
                        lockedAngle = newAngle;
                        lockedDist = newDist;
                        Serial.printf("🎯 TRACK re-locked at %d° %.1f cm\n", lockedAngle, lockedDist);
                    }
                    else
                    {
                        // ±10° ไม่เจอ → scan 0-180° เลย
                        Serial.println("🔍 TRACK full scan minRotate-,maxRotate");
                        int fullAngle = scanRange(minRotate, maxRotate);

                        if (fullAngle != -1)
                        {
                            radarServo.write(fullAngle);
                            vTaskDelay(pdMS_TO_TICKS(20));
                            float fullDist = getDistance();
                            lockedAngle = fullAngle;
                            lockedDist = fullDist;
                            Serial.printf("🎯 TRACK found at %d° %.1f cm\n", lockedAngle, lockedDist);
                        }
                        else
                        {
                            // หาไม่เจอเลย → กลับไป ROAM
                            tracking = false;
                            Serial.println("❌ TRACK lost target, back to ROAM");
                            vTaskDelay(pdMS_TO_TICKS(30));
                            continue;
                        }
                    }
                }
                else
                {
                    lockedDist = currentDist;
                }

                // report ตำแหน่งปัจจุบัน
                angle = lockedAngle;
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "{\"mode\":\"TRACK\",\"type\":\"%s\",\"angle\":%d,\"distance\":%.1f}",
                         getType(lockedDist), lockedAngle, lockedDist);
                mqttPublish("BeanNian/esp32/report", msg);
                Serial.printf("📡 TRACK [%d°] %.1f cm\n", lockedAngle, lockedDist);

                vTaskDelay(pdMS_TO_TICKS(30));
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