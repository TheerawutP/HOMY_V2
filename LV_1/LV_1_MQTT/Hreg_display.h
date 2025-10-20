#ifndef HREG_DISPLAY_H
#define HREG_DISPLAY_H

#include <ModbusRTU.h>

void printHoldingRegisters(ModbusRTU& rtuSlave, int k, String& output);

#endif
