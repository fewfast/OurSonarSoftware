#pragma once

void mqttTask(void *pvParameters);
void mqttPublish(const char *topic, const char *message);
