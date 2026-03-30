#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t mqttMutex;

void mqttTask(void *pvParameters);
void mqttPublish(const char *topic, const char *message);
void mqttGracefulDisconnect();