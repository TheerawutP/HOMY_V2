#include "Hreg_setting.h"
#include <Arduino.h>

void Hreg_setting_call(ModbusRTU& slave, uint8_t slave_id, uint8_t pin_en, uint8_t pin_rx, uint8_t pin_tx, int Hreg_lim) {
  pinMode(pin_en, OUTPUT);
  digitalWrite(pin_en, LOW);
  Serial2.begin(38400, SERIAL_8E1, pin_rx, pin_tx);
  slave.begin(&Serial2, pin_en, true);
  slave.slave(slave_id);

  for (uint16_t i = 0; i <= Hreg_lim; i++) {
    slave.addHreg(i);
    if (i == 0) {
      slave.onSetHreg(i, 0);
    }
    slave.onGetHreg(i, 0);
  }
}
