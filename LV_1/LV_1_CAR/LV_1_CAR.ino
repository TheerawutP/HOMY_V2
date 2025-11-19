#include <ModbusRTU.h>     // For Slave role (talking to STM32 Master)
#include <ModbusMaster.h>  // For Master role (talking to AIM/DOOR Slaves)
#include <math.h>

// --- MODBUS CONFIGURATION & SLAVE IDs ---
#define CAR_STA 1       // Slave ID for THIS device (when talking to STM32)
#define SENSOR_BOARD 1  // Slave ID for the DOOR peripheral on Bus B
#define AIM_BOARD 2     // Slave ID for the AIM peripheral on Bus B
#define CMD_BOARD 3

#define floor_ss_1 27
#define floor_ss_2 14
#define floor_ss_3 12
// #define floor_ss_4 23
// #define floor_ss_5 22
// #define floor_ss_6 19


#define DEBOUNCE_MS 50
// --- MODBUS OBJECTS ---
ModbusMaster node;    // Master object for Bus B (CAR -> AIM/DOOR)
ModbusRTU RTU_SLAVE;  // Slave object for Bus A (STM32 -> CAR)

// --- STATE MACHINE ---
enum ModbusState {
  STATE_IDLE,
  STATE_READ_SS,
  STATE_WRITE_SS,
  STATE_READ_AIM,
  STATE_WRITE_AIM,
  STATE_READ_CMD,
  STATE_WRITE_CMD
};

typedef struct {
  uint16_t wstate1;
  uint16_t wstate2;
  uint16_t wstate3;
} CMD;
CMD parsing;

QueueHandle_t xWriteQueue;
TaskHandle_t xUpdatePos;
TimerHandle_t xResetFrameTimer;
SemaphoreHandle_t xSemResetFrame;

ModbusState currentState = STATE_READ_SS;  // Start the cycle immediately

uint16_t lastSVal = 0;  // For HoldregSet callback

uint16_t IN_QUEUE = 0;  // Command word for Aiming_board
uint16_t ss_read = 0;   // Status read from SENSOR_BOARD
uint16_t aim_read = 0;  // Status read from AIM_BOARD
uint16_t cmd_read = 0;
uint16_t package = 0;
uint16_t aiming_frame = 0;

volatile uint32_t lastFloorISR_1 = 0;
volatile uint32_t lastFloorISR_2 = 0;
volatile uint32_t lastFloorISR_3 = 0;

uint16_t bit_s[16] = { 0 };
uint16_t bit_r1[16] = { 0 };
uint16_t bit_r2[16] = { 0 };
uint16_t bit_r3[16] = { 0 };
uint16_t bit_IN_QUEUE[16] = { 0 };

// ---------------------- CALLBACK FUNCTIONS ----------------------

/*
 * Modbus callback function: Executed when the STM32 Master writes to Hreg 0.
 * Logs the received value.
 */
uint16_t cbWrite(TRegister *reg, uint16_t val) {
  if (lastSVal != val) {
    lastSVal = val;
    // Serial.println(String("HregSet val (from STM32):") + String(val));
  }
  reg->value = val;
  return val;
}

/*
 * Modbus callback function: Executed when the STM32 Master reads Hreg 0 or 1.
 */
uint16_t cbRead(TRegister *reg, uint16_t val) {
  reg->value = val;
  return val;
}

/*
 * Helper function to set or clear a specific bit in a 16-bit word.
 */


void extractDataframe(uint16_t *frame, uint16_t val) {
  for (int i = 0; i <= 15; i++) {
    frame[i] = (val >> i) & 0x0001;
  }
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

// ---------------------- MASTER RESULT HANDLERS ----------------------

void handleMasterReadResult(uint8_t slaveId, uint8_t result, uint16_t &readVar) {
  if (result == node.ku8MBSuccess) {
    readVar = node.getResponseBuffer(0);  // Store the first register read
                                          // Serial.print("SUCCESS: Read ");
                                          // Serial.print(readVar);
                                          // Serial.print(" from ");
                                          // Serial.println(slaveName);
  } else {
    // Serial.print("ERROR: Read failed (Slave ");
    // Serial.print(slaveId);
    // Serial.print(", Code ");
    // Serial.print(result); // Check this code for timeouts (226) or invalid data (227)
    // Serial.print(")");
    // Serial.println();
  }
}

void handleMasterWriteResult(uint8_t slaveId, uint8_t result, uint16_t writeVal) {
  if (result == node.ku8MBSuccess) {
    // Serial.print("SUCCESS: Wrote ");
    // Serial.print(writeVal);
    // Serial.print(" to ");
    // Serial.println(slaveName);
  } else {
    // Serial.print("ERROR: Write failed (Slave ");
    // Serial.print(slaveId);
    // Serial.print(", Code ");
    // Serial.print(result);
    // Serial.print(")");
    // Serial.println();
  }
}

// ---------------------- SETUP ----------------------

void vModbusComTask(void *pvParameters) {
  for (;;) {
    RTU_SLAVE.task();
    uint16_t val = RTU_SLAVE.Hreg(0);
    uint16_t in_queue = RTU_SLAVE.Hreg(1);

    bit_s[0] = (val & 0x0001) != 0;  //take state of bit
    bit_s[1] = (val & 0x0002) != 0;
    bit_s[2] = (val & 0x0004) != 0;
    bit_s[3] = (val & 0x0008) != 0;
    //light and fan
    writeBit(parsing.wstate1, 9, bit_s[0]);
    //lock car RBL
    writeBit(parsing.wstate1, 6, bit_s[1]);
    writeBit(parsing.wstate1, 7, bit_s[1]);
    writeBit(parsing.wstate1, 8, bit_s[1]);
    //open car RBL
    writeBit(parsing.wstate1, 1, bit_s[2]);
    writeBit(parsing.wstate1, 3, bit_s[2]);
    writeBit(parsing.wstate1, 5, bit_s[2]);
    //close car RBL
    writeBit(parsing.wstate1, 0, bit_s[3]);
    writeBit(parsing.wstate1, 4, bit_s[3]);
    writeBit(parsing.wstate1, 2, bit_s[3]);

    bit_s[4] = (val & 0x0010) != 0;
    bit_s[5] = (val & 0x0020) != 0;
    bit_s[6] = (val & 0x0040) != 0;
    bit_s[7] = (val & 0x0080) != 0;
    bit_s[8] = (val & 0x0100) != 0;
    bit_s[9] = (val & 0x0200) != 0;
    //   bit_s[10] = (val & 0x0400) != 0;
    //   bit_s[11] = (val & 0x0800) != 0;
    //   bit_s[12] = (val & 0x1000) != 0;
    //   bit_s[13] = (val & 0x2000) != 0;

    //motion
    writeBit(parsing.wstate2, 0, bit_s[4]);
    writeBit(parsing.wstate2, 1, bit_s[5]);
    //car position
    writeBit(parsing.wstate2, 2, bit_s[6]);
    writeBit(parsing.wstate2, 3, bit_s[7]);
    writeBit(parsing.wstate2, 4, bit_s[8]);
    writeBit(parsing.wstate2, 5, bit_s[9]);
    //confirm aiming
    // writeBit(wstate2, 6, bit_s[10]);
    // writeBit(wstate2, 7, bit_s[11]);
    // writeBit(wstate2, 8, bit_s[12]);
    // writeBit(wstate2, 9, bit_s[13]);

    parsing.wstate3 = (ss_read >> 4);

    extractDataframe(bit_IN_QUEUE, in_queue);
    for (int i = 6; i <= 13; i++) {
      writeBit(parsing.wstate2, i, bit_IN_QUEUE[i - 6]);
    }

    RTU_SLAVE.Hreg(2, package);
    RTU_SLAVE.Hreg(3, aim_read);

    xQueueSend(xWriteQueue, &parsing, 0);
    vTaskDelay(10);
  }
}

void vUART_writeTask(void *pvParameters) {
  CMD cmd;
  uint16_t result;
  for (;;) {
    if (xQueueReceive(xWriteQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (currentState) {
        case STATE_READ_SS:
          node.begin(SENSOR_BOARD, Serial2);
          result = node.readHoldingRegisters(0x0001, 1);
          handleMasterReadResult(SENSOR_BOARD, result, ss_read);
          currentState = STATE_WRITE_SS;
          break;

        case STATE_WRITE_SS:
          node.begin(SENSOR_BOARD, Serial2);
          result = node.writeSingleRegister(0x0000, cmd.wstate1);
          handleMasterWriteResult(SENSOR_BOARD, result, cmd.wstate1);
          currentState = STATE_READ_AIM;
          break;

        case STATE_READ_AIM:
          node.begin(AIM_BOARD, Serial2);

          result = node.readHoldingRegisters(0x0001, 1);
          handleMasterReadResult(AIM_BOARD, result, aim_read);
          currentState = STATE_WRITE_AIM;
          break;

        case STATE_WRITE_AIM:
          node.begin(AIM_BOARD, Serial2);
          result = node.writeSingleRegister(0x0000, cmd.wstate2);
          handleMasterWriteResult(AIM_BOARD, result, cmd.wstate2);
          currentState = STATE_READ_CMD;
          break;

        case STATE_READ_CMD:
          node.begin(CMD_BOARD, Serial2);
          result = node.readHoldingRegisters(0x0001, 1);
          handleMasterReadResult(CMD_BOARD, result, cmd_read);
          currentState = STATE_WRITE_CMD;
          break;

        case STATE_WRITE_CMD:
          node.begin(CMD_BOARD, Serial2);
          result = node.writeSingleRegister(0x0000, cmd.wstate3);
          handleMasterWriteResult(CMD_BOARD, result, cmd.wstate3);
          currentState = STATE_IDLE;
          break;

        case STATE_IDLE:
          currentState = STATE_READ_SS;
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vUpdatePos(void *arg) {
  uint32_t val;
  for (;;) {
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &val, portMAX_DELAY) == pdTRUE) {
      //reg2 b5-8 for pos sent stm32
      writeBit(package, 5, val & 0x01);
      writeBit(package, 6, (val >> 1) & 0x01);
      writeBit(package, 7, (val >> 2) & 0x01);
      writeBit(package, 8, (val >> 3) & 0x01);
      xTimerStart(xResetFrameTimer, 0);
      }
    }
  }


void vResetFrameCallback(TimerHandle_t xTimer){
  package = 0;
}

void ARDUINO_ISR_ATTR ISR_atFloor1() {
  unsigned long now = millis();
  if (now - lastFloorISR_1 < DEBOUNCE_MS) return;  // debounce 50ms
  lastFloorISR_1 = now;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xUpdatePos, 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_atFloor2() {
  unsigned long now = millis();
  if (now - lastFloorISR_2 < DEBOUNCE_MS) return;  // debounce 50ms
  lastFloorISR_2 = now;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xUpdatePos, 2, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_atFloor3() {
  unsigned long now = millis();
  if (now - lastFloorISR_3 < DEBOUNCE_MS) return;  // debounce 50ms
  lastFloorISR_3 = now;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xUpdatePos, 3, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup() {
  Serial.begin(115200);

  Serial1.begin(38400, SERIAL_8E1, 15, 5);
  RTU_SLAVE.begin(&Serial1);
  RTU_SLAVE.slave(CAR_STA);

  Serial2.begin(38400, SERIAL_8E1, 16, 17);  // RX=16, TX=17
  node.begin(SENSOR_BOARD, Serial2);

  RTU_SLAVE.addHreg(0x0000, 0);
  RTU_SLAVE.onGetHreg(0x0000, cbRead);
  RTU_SLAVE.onSetHreg(0x0000, cbWrite);

  RTU_SLAVE.addHreg(0x0001, 0);
  RTU_SLAVE.onGetHreg(0x0001, cbRead);
  RTU_SLAVE.onSetHreg(0x0001, cbWrite);

  RTU_SLAVE.addHreg(0x0002, 0);
  RTU_SLAVE.onGetHreg(0x0002, cbRead);
  RTU_SLAVE.onSetHreg(0x0002, cbWrite);

  RTU_SLAVE.addHreg(0x0003, 0);
  RTU_SLAVE.onGetHreg(0x0003, cbRead);
  RTU_SLAVE.onSetHreg(0x0003, cbWrite);

  pinMode(floor_ss_1, INPUT_PULLUP);
  pinMode(floor_ss_2, INPUT_PULLUP);
  pinMode(floor_ss_3, INPUT_PULLUP);
  attachInterrupt(floor_ss_1, ISR_atFloor1, FALLING);
  attachInterrupt(floor_ss_2, ISR_atFloor2, FALLING);
  attachInterrupt(floor_ss_3, ISR_atFloor3, FALLING);

  //xSemResetFrame = xSemaphoreCreateBinary();
  //xSemaphoreGive(xSemResetFrame);
  xResetFrameTimer = xTimerCreate("ResetFrameTime", pdMS_TO_TICKS(500), pdFALSE, 0, vResetFrameCallback);
  xWriteQueue = xQueueCreate(16, sizeof(parsing));
  xTaskCreate(vUART_writeTask, "UART_Write", 2048, NULL, 3, NULL);
  xTaskCreate(vModbusComTask, "ModbusCom", 2048, NULL, 3, NULL);
  xTaskCreate(vUpdatePos, "UpdatePos", 1024, NULL, 3, &xUpdatePos);
}


void loop() {
  Serial.println(aim_read);
  Serial.println(RTU_SLAVE.Hreg(0));
  Serial.println(RTU_SLAVE.Hreg(1));
  Serial.println(RTU_SLAVE.Hreg(2));
  Serial.println(RTU_SLAVE.Hreg(3));
  Serial.println("----------------------");

  // if (Serial1.available()) {
  //     Serial.print("RAW RX: ");
  //     while (Serial1.available()) {
  //       Serial.print(Serial1.read(), HEX);
  //       Serial.print(" ");
  //     }
  //     Serial.println();
  //   }

  vTaskDelay(pdMS_TO_TICKS(1000));
}
