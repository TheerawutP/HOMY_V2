#include <ModbusRTU.h>
#include <math.h>

#define ss_bit0 36
#define ss_bit1 39
#define ss_bit2 34
#define ss_bit3 35

#define segB0 32
#define segB1 33
#define segB2 25
#define segB3 26

#define out1 12  //button
#define out2 13
#define out3 23
#define out4 22
#define out5 19
#define out6 18

#define DOWN 4
#define UP 5

QueueHandler_t xDisplayQueue;

ModbusRTU RTU_SLAVE;
uint16_t lastSVal;

unsigned long last = 0;
int h = 0;

//callback for handle writing at Hreg
uint16_t cbWrite(TRegister* reg, uint16_t val) {
  // if (lastSVal != val) {
  //   lastSVal = val;
  //   Serial.println(String("HregSet val:") + String(val));
  // }
  return val;
}

//callback for handle reading at Hreg
uint16_t cbRead(TRegister* reg, uint16_t val) {
  reg->value = val;   
  return val;
}

//callback for handle reading at Hreg
void writeBit(uint16_t &value, uint8_t bit, bool state) {
    if (state) {
        value |= (1 << bit);     // set
    } else {
        value &= ~(1 << bit);    // clear
    }
}

void extractDataframe(uint16_t *frame[16], uint16_t val){
  for(int i = 0; i<=15; i++){
    frame[i] = (val >> i) & 0x0001;
  }
}

void setup() {
  Serial.begin(115200);
    //setup as master
  Serial2.begin(38400, SERIAL_8E1, 16, 17); // RX=16, TX=17
    //setup as slave 
  RTU_SLAVE.begin(&Serial2);
  RTU_SLAVE.slave(2);      
  
  //initiate Hreg
  RTU_SLAVE.addHreg(0x0000,0);   //for write by main controller 
  RTU_SLAVE.onSetHreg(0x0000, cbWrite); //written enable   
  RTU_SLAVE.onGetHreg(0x0000, cbRead);
  
  RTU_SLAVE.addHreg(0x0001,0);   //for read by main controller 
  RTU_SLAVE.onSetHreg(0x0001, cbWrite); 
  RTU_SLAVE.onGetHreg(0x0001, cbRead); 

  pinMode(ss_bit0, INPUT_PULLUP);   
  pinMode(ss_bit1, INPUT_PULLUP); 
  // pinMode(ss_bit0, INPUT);   
  // pinMode(ss_bit1, INPUT);    
  pinMode(ss_bit2, INPUT);    
  pinMode(ss_bit2, INPUT);   

  pinMode(segB0, OUTPUT);   
  pinMode(segB1, OUTPUT);   
  pinMode(segB2, OUTPUT);    
  pinMode(segB3, OUTPUT);    

  pinMode(DOWN, OUTPUT);    
  pinMode(UP, OUTPUT);  

  xDisplayQueue = xQueueCreate(8, sizeof(ButtonEvent_t));  //handler for evnet queue
  xTaskCreate(vAimTask, "Aim_Scheduling", 2048, NULL, 3, NULL);

  for(int i = 1; i<= MAX_FL; i++){
    
  }

}

void vModbusComTask(void *pvParameters){
  for(;;){
  RTU_SLAVE.task();
  uint16_t val = RTU_SLAVE.Hreg(0);
  extractDataframe(&parsing_data, val);
  /*
  6 - 13 waiting in queue fl1 - 8 (turn on lamp fl1-8)

  5 car pos b3
  4 car pos b2
  3 car pos b1
  2 car pos b0

  1 on up lamp
  0 on dw lamp
  */
  xQueueSend(xDisplayHandle, &parsing_data);
  vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vAimTask(void *pvParameters){
  for(;;){
      uint8_t target = 0;
      encode_aiming(target);
      if(target != 0){
        writeBit(package, 0, 1);                                  //dont forget to set 0 by STM after complete all cmd
        writeBit(package, target, 1);
        xTimerStart(xHoldStateTimer, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}
  
void vDisplayTask(){
  uint16_t state[16];
  for(;;){

    if(xQueueRecieve(xDisplayQueue, &state, portMAX_DEKAY) == pdTrue){
      
      DW_LAMP(state[0]);
      UP_LAMP(state[1]);
      
      uint8_t pos = state[2] + (state[3]*2) + (state[4]*4) + (state[5]*8);
      POS_SEGMENTS(pos);

      for(int i = 6; i <= 13; i++){
          WAIT_IN_QUEUE(state[i]);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void loop() {
 
}



