#include "state.h"

volatile bool isSystemStart = true;
volatile bool isAlert = false;
volatile int pos = 0;
volatile int minRotate = 0;
volatile int maxRotate = 180;
volatile int maxRange = 400;
volatile int RedZone = 100;
volatile char mode[16] = "ROAM";