#include "Hreg_display.h"
#include <Arduino.h>

// void printHoldingRegisters(ModbusRTU& rtuSlave, int Hreg_lim) {
//   Serial.println();
//   for (int k = 0; k <= Hreg_lim; k++) {
//     uint16_t regValue = rtuSlave.Hreg(k);

//     // พิมพ์ index ของ register
//     Serial.print("Holding_register : ");
//     Serial.print(k);
//     Serial.print(" = ");

//     // แสดงเป็น binary 16-bit
//     for (int i = 15; i >= 0; i--) {
//       Serial.print((regValue >> i) & 1);
//     }

//     // แสดงค่า decimal
    // Serial.print(" (");
    // Serial.print(regValue);
    // Serial.println(")");
//   }
//   Serial.println();
// }


void printHoldingRegisters(ModbusRTU& rtuSlave, int k, String& output) {
  output = "";  // Clear previous content

    uint16_t regValue = rtuSlave.Hreg(k);

    Serial.print("Holding_register : ");
    Serial.print(k);
    Serial.print(" : ");

    // Append 16-bit binary representation
    for (int i = 15; i >= 0; i--) {
      output += String((regValue >> i) & 1);
    }
    
    Serial.print("BIN : ");
    Serial.print(output);

    Serial.print(" (");
    Serial.print(regValue);
    Serial.println(")");
}

