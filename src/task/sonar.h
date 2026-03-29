#pragma once

#include <Arduino.h>

extern volatile bool alertEnabled;
extern volatile bool servoEnabled;

void sonarTask(void *pvParameters);