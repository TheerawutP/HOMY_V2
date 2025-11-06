#include <ModbusRTU.h>    // For Slave role (talking to STM32 Master)
#include <ModbusMaster.h> // For Master role (talking to AIM/DOOR Slaves)
#include <math.h>

// --- MODBUS CONFIGURATION & SLAVE IDs ---
#define CAR_STA 1      // Slave ID for THIS device (when talking to STM32)
#define SENSOR_BOARD 1      // Slave ID for the DOOR peripheral on Bus B
#define AIM_BOARD 2       // Slave ID for the AIM peripheral on Bus B
#define CMD_BOARD 3

// --- MODBUS OBJECTS ---
ModbusMaster node;     // Master object for Bus B (CAR -> AIM/DOOR)
ModbusRTU RTU_SLAVE;   // Slave object for Bus A (STM32 -> CAR)

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

ModbusState currentState = STATE_READ_SS; // Start the cycle immediately
unsigned long nextActionTime = 0;
const unsigned long ACTION_INTERVAL = 20;    // Time (ms) to wait between full cycles
const unsigned long DELAY_BETWEEN_SLAVES = 20; // Short gap (ms) after a transaction finishes

// --- GLOBAL VARIABLES (Holding data and status) ---
uint16_t lastSVal = 0; // For HoldregSet callback

// Data to be written to peripheral slaves
uint16_t wstate1 = 0; // Command word for SENSOR_BOARD
uint16_t wstate2 = 0; // Command word for AIM_BOARD
uint16_t wstate3 = 0; // Command word for CMD_BOARD
uint16_t IN_QUEUE = 0; // Command word for Aiming_board

// Data read from peripheral slaves
uint16_t ss_read = 0; // Status read from SENSOR_BOARD
uint16_t aim_read = 0; // Status read from AIM_BOARD
uint16_t cmd_read =0;

// Response package for the STM32 Master
uint16_t package = 0; 
uint16_t aiming_frame = 0; 

unsigned long now = 0;
uint32_t time_print;
uint32_t last_time_print = 0;
uint32_t print_interval = 1000;
uint16_t bit_s[16] = {0};
uint16_t bit_r1[16] = {0};
uint16_t bit_r2[16] = {0};
uint16_t bit_r3[16] = {0};
uint16_t bit_IN_QUEUE[16] = {0};

// ---------------------- CALLBACK FUNCTIONS ----------------------

/*
 * Modbus callback function: Executed when the STM32 Master writes to Hreg 0.
 * Logs the received value.
 */
uint16_t cbWrite(TRegister* reg, uint16_t val) {
//   if (lastSVal != val) {
//     lastSVal = val;
//     Serial.println(String("HregSet val (from STM32):") + String(val));
//   }
  return val;
}

/*
 * Modbus callback function: Executed when the STM32 Master reads Hreg 0 or 1.
 */
uint16_t cbRead(TRegister* reg, uint16_t val) {
  reg->value = val;   
  return val;
}

/*
 * Helper function to set or clear a specific bit in a 16-bit word.
 */


void extractDataframe(uint16_t *frame, uint16_t val){
  for(int i = 0; i<=15; i++){
    frame[i] = (val >> i) & 0x0001;
  }
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
    if (state) {
        value |= (1 << bit);      // set
    } else {
        value &= ~(1 << bit);     // clear
    }
}

// ---------------------- MASTER RESULT HANDLERS ----------------------

void handleMasterReadResult(uint8_t slaveId, uint8_t result, uint16_t& readVar, const char* slaveName) {
    if (result == node.ku8MBSuccess) {
        readVar = node.getResponseBuffer(0); // Store the first register read
        Serial.print("SUCCESS: Read ");
        Serial.print(readVar);
        Serial.print(" from ");
        Serial.println(slaveName);
    } else {
        Serial.print("ERROR: Read failed (Slave ");
        Serial.print(slaveId);
        Serial.print(", Code ");
        Serial.print(result); // Check this code for timeouts (226) or invalid data (227)
        Serial.print(")");
        Serial.println();
    }
}

void handleMasterWriteResult(uint8_t slaveId, uint8_t result, uint16_t writeVal, const char* slaveName) {
    if (result == node.ku8MBSuccess) {
        Serial.print("SUCCESS: Wrote ");
        Serial.print(writeVal);
        Serial.print(" to ");
        Serial.println(slaveName);
    } else {
        Serial.print("ERROR: Write failed (Slave ");
        Serial.print(slaveId);
        Serial.print(", Code ");
        Serial.print(result);
        Serial.print(")");
        Serial.println();
    }
}

// ---------------------- SETUP ----------------------

void setup() {
  Serial.begin(115200);
  
  // Serial1 (Bus A): SLAVE port (talking to STM32 Master)
  Serial1.begin(38400, SERIAL_8E1, 15, 5); // RX=9, TX=10 (Example pins for ESP32)
  RTU_SLAVE.begin(&Serial1);
  RTU_SLAVE.slave(CAR_STA); // Slave ID 1

  // Serial2 (Bus B): MASTER port (talking to SENSOR_BOARD and AIM_BOARD Slaves)
  Serial2.begin(38400, SERIAL_8E1, 16, 17); // RX=16, TX=17
  
  // ModbusMaster initializes the master on the bus. Slave ID is set dynamically in loop.
  // Note: We initialize with a placeholder ID; the actual slave ID is set via node.begin(ID) in the loop.
  node.begin(SENSOR_BOARD, Serial2); 

  // SLAVE Register Setup for STM32 Master
  // Hreg 0x0000: Command register (Write by STM32, Read by CAR)
  RTU_SLAVE.addHreg(0x0000, 0); 
RTU_SLAVE.onGetHreg(0x0000, cbRead); 
  RTU_SLAVE.onSetHreg(0x0000, cbWrite); 
  
  //RTU_SLAVE.onGetHreg(0x0000, cbWrite);
  
  // Hreg 0x0001: Status register (Write by CAR, Read by STM32)
  RTU_SLAVE.addHreg(0x0001, 0);  
  //RTU_SLAVE.onSetHreg(0x0001, HoldregSet); 
  RTU_SLAVE.onGetHreg(0x0001, cbRead); 
  RTU_SLAVE.onSetHreg(0x0001, cbWrite); 

  RTU_SLAVE.addHreg(0x0002, 0);  
  RTU_SLAVE.onGetHreg(0x0002, cbRead); 
  RTU_SLAVE.onSetHreg(0x0002, cbWrite); 

  RTU_SLAVE.addHreg(0x0003, 0);  
  RTU_SLAVE.onGetHreg(0x0003, cbRead); 
  RTU_SLAVE.onSetHreg(0x0003, cbWrite); 


    nextActionTime = millis();
}

// ---------------------- LOOP (MAIN EXECUTION) ----------------------

void loop() {
  now = millis();
  time_print = millis();
  RTU_SLAVE.task(); 

  // Read the latest command from the STM32 Master (Hreg 0)
  uint16_t val = RTU_SLAVE.Hreg(0); 
  uint16_t in_queue = RTU_SLAVE.Hreg(1);
  // **Application Logic:** Map the STM32 command (val) to the slave command words (wstate1, wstate2).
  // This logic runs continuously so wstate1/wstate2 are always ready when a write state is reached.
  
  wstate1 = 0; // Reset DOOR command
  wstate2 = 0; // Reset AIM command
  wstate3 = 0; // Reset CMD command

  bit_s[0] = (val & 0x0001) != 0;            //take state of bit
  bit_s[1] = (val & 0x0002) != 0;
  bit_s[2] = (val & 0x0004) != 0;
  bit_s[3] = (val & 0x0008) != 0;
  //light and fan
  writeBit(wstate1, 9, bit_s[0]); 
  //lock car RBL
  writeBit(wstate1, 6, bit_s[1]); 
  writeBit(wstate1, 7, bit_s[1]); 
  writeBit(wstate1, 8, bit_s[1]); 
  //open car RBL
  writeBit(wstate1, 1, bit_s[2]); 
  writeBit(wstate1, 3, bit_s[2]); 
  writeBit(wstate1, 5, bit_s[2]); 
  //close car RBL
  writeBit(wstate1, 0, bit_s[3]); 
  writeBit(wstate1, 4, bit_s[3]); 
  writeBit(wstate1, 2, bit_s[3]); 

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
  writeBit(wstate2, 0, bit_s[4]); 
  writeBit(wstate2, 1, bit_s[5]); 
  //car position
  writeBit(wstate2, 2, bit_s[6]); 
  writeBit(wstate2, 3, bit_s[7]); 
  writeBit(wstate2, 4, bit_s[8]); 
  writeBit(wstate2, 5, bit_s[9]); 
  //confirm aiming
  writeBit(wstate2, 6, bit_s[10]); 
  writeBit(wstate2, 7, bit_s[11]); 
  writeBit(wstate2, 8, bit_s[12]); 
  writeBit(wstate2, 9, bit_s[13]); 

  wstate3 = (ss_read>>4);

  extractDataframe(bit_IN_QUEUE, in_queue);
  for(int i = 6; i<=13; i++){
    writeBit(wstate2, i, bit_IN_QUEUE[i-6]);
  }

  
  // 2. Check if it's time for the next Master action
  if (now < nextActionTime) {
      return; // CPU returns here to keep calling RTU_SLAVE.task()
  }

  uint8_t result;

  // 3. Modbus Master State Machine (Politely communicating with AIM/DOOR)
  switch (currentState) {
    case STATE_READ_SS: // Read Hreg 1 from SENSOR_BOARD (Slave ID 1)
        node.begin(SENSOR_BOARD, Serial2);
        result = node.readHoldingRegisters(0x0001, 1);
        handleMasterReadResult(SENSOR_BOARD, result, ss_read, "SENSOR_BOARD");
        currentState = STATE_WRITE_SS;
        nextActionTime = now + DELAY_BETWEEN_SLAVES; 
        break;

    case STATE_WRITE_SS: // Write Hreg 0 to SENSOR_BOARD (Slave ID 1)
        node.begin(SENSOR_BOARD, Serial2);
        result = node.writeSingleRegister(0x0000, wstate1); 
        handleMasterWriteResult(SENSOR_BOARD, result, wstate1, "SENSOR_BOARD");
        currentState = STATE_READ_AIM;
        nextActionTime = now + DELAY_BETWEEN_SLAVES;
        break;
        
    case STATE_READ_AIM: // Read Hreg 1 from AIM_BOARD (Slave ID 2)
        node.begin(AIM_BOARD, Serial2);
        result = node.readHoldingRegisters(0x0001, 1);
        handleMasterReadResult(AIM_BOARD, result, aim_read, "AIM_BOARD");
        currentState = STATE_WRITE_AIM;
        nextActionTime = now + DELAY_BETWEEN_SLAVES; 
        break;

    case STATE_WRITE_AIM: // Write Hreg 0 to AIM_BOARD (Slave ID 2)
        node.begin(AIM_BOARD, Serial2);
        result = node.writeSingleRegister(0x0000, wstate2);
        handleMasterWriteResult(AIM_BOARD, result, wstate2, "AIM_BOARD");
        // Cycle back to the start of the polling loop
        currentState = STATE_READ_CMD; 
        nextActionTime = now + DELAY_BETWEEN_SLAVES;
        break;

    case STATE_READ_CMD: 
        node.begin(CMD_BOARD, Serial2);
        result = node.readHoldingRegisters(0x0001, 1);
        handleMasterReadResult(CMD_BOARD, result, cmd_read, "CMD_BOARD");
        currentState = STATE_WRITE_CMD;
        nextActionTime = now + DELAY_BETWEEN_SLAVES; 
        break;

    case STATE_WRITE_CMD: 
        node.begin(CMD_BOARD, Serial2);
        result = node.writeSingleRegister(0x0000, wstate3); 
        handleMasterWriteResult(CMD_BOARD, result, wstate3, "CMD_BOARD");
        currentState = STATE_IDLE;
        nextActionTime = now + ACTION_INTERVAL;
        break;
    
    case STATE_IDLE: 
        // Wait for ACTION_INTERVAL before starting the next full cycle
        currentState = STATE_READ_SS;
        // nextActionTime is set in the previous state.
        break;
  }
  
  // 4. PACK RESPONSE DATA FOR STM32 MASTER (Hreg 1)
  // Combine status from all local slaves (ss_read, aim_read) into one package for the STM32.
  package = 0; 
  aiming_frame = 0;
  bit_r1[0] = (ss_read & 0x0001) != 0;   
  writeBit(package, 2, bit_r1[0]); 
  
  bit_r1[10] = (ss_read & 0x0400) != 0; 
  bit_r1[11] = (ss_read & 0x0800) != 0;
  bit_r1[12] = (ss_read & 0x1000) != 0;
  writeBit(package, 0, bit_r1[10] | bit_r1[11] | bit_r1[12] ); 

  bit_r1[1] = (ss_read & 0x0002) != 0;
  bit_r1[2] = (ss_read & 0x0004) != 0;
  bit_r1[3] = (ss_read & 0x0008) != 0;
  writeBit(package, 1, bit_r1[1] | bit_r1[2] | bit_r1[3]); 

  // //read and store in package from aiming board
  bit_r2[0] = (aim_read & 0x0001) != 0;  
  bit_r2[1] = (aim_read & 0x0002) != 0;  
  bit_r2[2] = (aim_read & 0x0004) != 0;  
  bit_r2[3] = (aim_read & 0x0008) != 0;  
  bit_r2[4] = (aim_read & 0x0010) != 0;  
  bit_r2[5] = (aim_read & 0x0020) != 0;  
  bit_r2[6] = (aim_read & 0x0040) != 0;  
  bit_r2[7] = (aim_read & 0x0080) != 0;  
  bit_r2[8] = (aim_read & 0x0100) != 0;  
  
  writeBit(aiming_frame, 0, bit_r2[0]); 
  writeBit(aiming_frame, 1, bit_r2[1]); 
  writeBit(aiming_frame, 2, bit_r2[2]); 
  writeBit(aiming_frame, 3, bit_r2[3]); 
  writeBit(aiming_frame, 4, bit_r2[4]); 
  writeBit(aiming_frame, 5, bit_r2[5]); 
  writeBit(aiming_frame, 6, bit_r2[6]); 
  writeBit(aiming_frame, 7, bit_r2[7]); 
  writeBit(aiming_frame, 8, bit_r2[8]); 

  //packing data from cmd_read to package
  bit_r3[0] = (cmd_read & 0x0001) != 0;
  bit_r3[1] = (cmd_read & 0x0002) != 0;
  bit_r3[2] = (cmd_read & 0x0004) != 0;
  bit_r3[3] = (cmd_read & 0x0008) != 0;
  bit_r3[4] = (cmd_read & 0x0010) != 0;
  bit_r3[5] = (cmd_read & 0x0020) != 0;
  writeBit(package, 9, bit_r3[1]); 
  writeBit(package, 10, bit_r3[2]); 
  writeBit(package, 11, bit_r3[3]); 
  writeBit(package, 12, bit_r3[4]); 
  writeBit(package, 13, bit_r3[5]); 

  RTU_SLAVE.Hreg(2, package);
  RTU_SLAVE.Hreg(3, aiming_frame);
  if(time_print - last_time_print >= print_interval){
    last_time_print = time_print;
    Serial.println(" ");
    Serial.print("Hreg[0]: "); Serial.println(RTU_SLAVE.Hreg(0));
    Serial.print("Hreg[1]: "); Serial.println(RTU_SLAVE.Hreg(1));
    Serial.print("Hreg[2]: "); Serial.println(RTU_SLAVE.Hreg(2));
    Serial.print("Hreg[3]: "); Serial.println(RTU_SLAVE.Hreg(3));

    Serial.println(" ");   
  }

}
