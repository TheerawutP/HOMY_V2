#ifndef HREG_SETTING_H
#define HREG_SETTING_H

#include <ModbusRTU.h>

void Hreg_setting_call(ModbusRTU& slave, uint8_t slave_id, uint8_t pin_en, uint8_t pin_rx, uint8_t pin_tx, int Hreg_lim);

#endif
