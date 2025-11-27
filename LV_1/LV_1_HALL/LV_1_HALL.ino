#include <ModbusRTU.h>     // For Slave role (talking to STM32 Master)
#include <ModbusMaster.h>  // For Master role (talking to AIM/DOOR Slaves)
#include <math.h>

//#define JOG 20
// --- MODBUS OBJECTS ---
ModbusMaster node;    // Master object for Bus B (CAR -> AIM/DOOR)
ModbusRTU RTU_SLAVE;  // Slave object for Bus A (STM32 -> CAR)

// --- STATE MACHINE ---
enum mb_state {
  STATE_READ,
  STATE_WRITE,
  STATE_IDLE
};

typedef struct {
  uint8_t current_slave;
  mb_state current_state;
} modbus_state;

modbus_state STATE = { .current_slave = 1, .current_state = STATE_READ };


#define slaveNum 3  //excluding remote jog
uint16_t write_frame[slaveNum + 1] = { 0 };
uint16_t read_frame[slaveNum + 1] = { 0 };


uint16_t lastSVal = 0;  // For HoldregSet callback

uint16_t package = 0;
uint16_t up_frame = 0;
uint16_t down_frame = 0;

uint16_t bit_s[16][16] = { 0 };
uint16_t bit_r[16][16] = { 0 };
//uint16_t bit_r2[16] = {0};

uint16_t cbWrite(TRegister* reg, uint16_t val) {
  // if (lastSVal != val) {
  //   lastSVal = val;
  //   Serial.println(String("HregSet val (from STM32):") + String(val));
  // }
  reg->value = val;
  return val;
}


uint16_t cbRead(TRegister* reg, uint16_t val) {
  reg->value = val;
  return val;
}

void writeBit(uint16_t& value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

// ---------------------- MASTER RESULT HANDLERS ----------------------

void handleMasterReadResult(uint8_t slaveId, uint8_t result, uint16_t& readVar) {
  if (result == node.ku8MBSuccess) {
    readVar = node.getResponseBuffer(0);  // Store the first register read
                                          // Serial.print("SUCCESS: Read ");
                                          // Serial.print(readVar);
                                          // Serial.print(" from slave: ");
                                          // Serial.println(slaveId);
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
    // Serial.print(" to slave: ");
    // Serial.println(slaveId);
  } else {
    // Serial.print("ERROR: Write failed (Slave ");
    // Serial.print(slaveId);
    // Serial.print(", Code ");
    // Serial.print(result);
    // Serial.print(")");
    // Serial.println();
  }
}


void vModbusSlaveTask(void* arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);
  for (;;) {

    RTU_SLAVE.task();

    uint16_t val[4];
    val[0] = RTU_SLAVE.Hreg(0);
    val[1] = RTU_SLAVE.Hreg(1);
    val[2] = RTU_SLAVE.Hreg(2);
    val[3] = RTU_SLAVE.Hreg(3);

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 16; j++) {
        bit_s[i][j] = ((val[i] >> j) & 0x0001) != 0;
      }
    }

    //lock/open/close command write to each
    for (int i = 1; i <= slaveNum; i++) {
      writeBit(write_frame[i], 0, bit_s[3][0]);  //write direction down
      writeBit(write_frame[i], 1, bit_s[3][1]);  //write direction up

      writeBit(write_frame[i], 2, bit_s[3][2]);  //confirm calling floor in 4-bit
      writeBit(write_frame[i], 3, bit_s[3][3]);
      writeBit(write_frame[i], 4, bit_s[3][4]);
      writeBit(write_frame[i], 5, bit_s[3][5]);

      writeBit(write_frame[i], 6, bit_s[3][6]);  //carPos in 4-bit
      writeBit(write_frame[i], 7, bit_s[3][7]);
      writeBit(write_frame[i], 8, bit_s[3][8]);
      writeBit(write_frame[i], 9, bit_s[3][9]);

      writeBit(write_frame[i], 10, bit_s[0][i - 1]);  //lock
      writeBit(write_frame[i], 11, bit_s[1][i - 1]);  //open
      writeBit(write_frame[i], 12, bit_s[2][i - 1]);  //close
    }


    for (int i = 1; i <= slaveNum; i++) {
      for (int j = 0; j < 16; j++) {
        bit_r[i][j] = ((read_frame[i] >> j) & 0x0001) != 0;
      }
    }

    for (int i = 1; i <= slaveNum; i++) {
      // if(bit_r[i][8] == 1){
      //int sum = (bit_r[i][4] +  bit_r[i][4]*2 +  bit_r[i][4]*4 +  bit_r[i][4]*8)-1;
      writeBit(package, 0, bit_r[i][8]);
      writeBit(up_frame, i, bit_r[i][9]);
      writeBit(down_frame, i, bit_r[i][10]);
      // }
    }

    RTU_SLAVE.Hreg(4, package);
    RTU_SLAVE.Hreg(5, up_frame);
    RTU_SLAVE.Hreg(6, down_frame);
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}


void vModbusMasterTask(void* arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);
  uint8_t result;
  for (;;) {
    switch (STATE.current_state) {
      case STATE_READ:
        node.begin(STATE.current_slave, Serial2);
        result = node.readHoldingRegisters(0x0001, 1);
        handleMasterReadResult(STATE.current_slave, result, read_frame[STATE.current_slave]);
        STATE.current_state = STATE_WRITE;
        break;

      case STATE_WRITE:
        node.begin(STATE.current_slave, Serial2);
        result = node.writeSingleRegister(0x0000, write_frame[STATE.current_slave]);
        handleMasterWriteResult(STATE.current_slave, result, write_frame[STATE.current_slave]);
        STATE.current_state = STATE_READ;
        
        if(STATE.current_slave == slaveNum){
            STATE.current_slave = 1;
        }else{
            STATE.current_slave += 1; 
        }
        break;

    }
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8E1, 15, 5);
  RTU_SLAVE.begin(&Serial1);
  RTU_SLAVE.slave(2);  // Slave ID 1

  //as master
  Serial2.begin(38400, SERIAL_8E1, 16, 17);  // RX=16, TX=17

  node.begin(STATE.current_slave, Serial2);

  //for written by main controller's data
  RTU_SLAVE.addHreg(0x0000, 0);
  RTU_SLAVE.onSetHreg(0x0000, cbWrite);
  RTU_SLAVE.onGetHreg(0x0000, cbRead);

  RTU_SLAVE.addHreg(0x0001, 0);
  RTU_SLAVE.onSetHreg(0x0001, cbWrite);
  RTU_SLAVE.onGetHreg(0x0001, cbRead);

  RTU_SLAVE.addHreg(0x0002, 0);
  RTU_SLAVE.onSetHreg(0x0002, cbWrite);
  RTU_SLAVE.onGetHreg(0x0002, cbRead);

  RTU_SLAVE.addHreg(0x0003, 0);
  RTU_SLAVE.onSetHreg(0x0003, cbWrite);
  RTU_SLAVE.onGetHreg(0x0003, cbRead);

  //for read
  RTU_SLAVE.addHreg(0x0004, 0);
  RTU_SLAVE.onSetHreg(0x0004, cbWrite);
  RTU_SLAVE.onGetHreg(0x0004, cbRead);

  RTU_SLAVE.addHreg(0x0005, 0);
  RTU_SLAVE.onSetHreg(0x0005, cbWrite);
  RTU_SLAVE.onGetHreg(0x0005, cbRead);

  RTU_SLAVE.addHreg(0x0006, 0);
  RTU_SLAVE.onSetHreg(0x0006, cbWrite);
  RTU_SLAVE.onGetHreg(0x0006, cbRead);

  xTaskCreate(vModbusSlaveTask, "SlaveTask", 2048, NULL, 4, NULL);
  xTaskCreate(vModbusMasterTask, "MasterTask", 2048, NULL, 3, NULL);
}

void loop() {

  // if (time_print - last_time_print >= print_interval) {
  //   last_time_print = time_print;
  //   Serial.println(" ");
    Serial.println(RTU_SLAVE.Hreg(0));
    Serial.println(RTU_SLAVE.Hreg(1));
    Serial.println(RTU_SLAVE.Hreg(2));
    Serial.print("Hreg[4] jog_frame: ");
    Serial.println(RTU_SLAVE.Hreg(4));
    Serial.print("Hreg[5] up_frmae: ");
    Serial.println(RTU_SLAVE.Hreg(5));
    Serial.print("Hreg[6] down_frame: ");
    Serial.println(RTU_SLAVE.Hreg(6));
    Serial.println("-------------");
  vTaskDelay(1000);
  // }
}
