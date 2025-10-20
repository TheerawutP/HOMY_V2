#ifndef AUTO_RESET_H
#define AUTO_RESET_H

#include <Arduino.h>

void AutoReset_init(unsigned long resetIntervalMin);
void AutoReset_loop();

#endif
