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

QueueHandler_t xAimQueue;

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

  xAimnQueue = xQueueCreate(16, sizeof(ButtonEvent_t));  //handler for evnet queue
  xTaskCreate(vAimTask, "Aim_Scheduling", 2048, NULL, 3, NULL);

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
  Aiming_t floor;
  for(;;){
      if(xQueueRecieve(xAimQueue, &floor, portMAX_DEKAY) == pdTrue){
          //packing target in dataframe 
          //display 
      }
  }
}
  
void vDisplay(){
  if(){
    
  }
}

void loop() {
  uint16_t package;
  uint16_t bit_r[16];
  
  RTU_SLAVE.task();
  ///////////////////////////////////////////////////////extract data from Hreg 0 (written by LV1)//////////////////////////////////////////

  uint16_t val = RTU_SLAVE.Hreg(0);
  //motion
  bit_r[0] = (val & 0x0001) != 0;            
  bit_r[1] = (val & 0x0002) != 0;    
  //car position
  bit_r[2] = (val & 0x0004) != 0;            
  bit_r[3] = (val & 0x0008) != 0;            
  bit_r[4] = (val & 0x0010) != 0;            
  bit_r[5] = (val & 0x0020) != 0;
  //confirm 
  bit_r[6] = (val & 0x0040) != 0;            
  bit_r[7] = (val & 0x0080) != 0;            
  bit_r[8] = (val & 0x0100) != 0;            
  bit_r[9] = (val & 0x0200) != 0;            

  /////////////////////////////////////////////////////decode 4-bit to 0-9/////////////////////////////////////////////////////////////////////
  

  // uint16_t decode; 
  // writeBit(decode, 0, ss_bit0);
  // writeBit(decode, 1, ss_bit1);
  // writeBit(decode, 2, ss_bit2);
  // writeBit(decode, 3, ss_bit3);

  // //from bit2-5 (position) in Hreg 0, light up 7-segment
  // if(ss_bit0 == bit_r[6]) digitalWrite(segB0, HIGH);
  // if(ss_bit1 == bit_r[7]) digitalWrite(segB1, HIGH);
  // if(ss_bit2 == bit_r[8]) digitalWrite(segB2, HIGH);
  // if(ss_bit3 == bit_r[9]) digitalWrite(segB3, HIGH);

  // //Up, Down or Stay, then light up arrows LED
  // if(bit_r[0] == 1 && bit_r[1] == 0){
  //   digitalWrite(DOWN, HIGH);
  // }else if(bit_r[0] == 0 && bit_r[1] == 1){
  //   digitalWrite(UP, HIGH);
  // }else{
  //   digitalWrite(DOWN, LOW);
  //   digitalWrite(UP, LOW);
  // }
  
  // //light up floor button  
  // switch (decode) {
  //     case 1:
  //       digitalWrite(out1, HIGH);
  //       break;

  //     case 2:
  //       digitalWrite(out2, HIGH);
  //       break;

  //     case 3:
  //       digitalWrite(out3, HIGH);
  //       break;

  //     case 4:
  //       digitalWrite(out4, HIGH);
  //       break;
        
  //     case 5:
  //       digitalWrite(out5, HIGH);
  //       break;

  //     case 6:
  //       digitalWrite(out6, HIGH);
  //       break;
  //   } 

  if(digitalRead(36) == LOW){
    writeBit(package, 1, 1);
  }else{
    writeBit(package, 1, 0);
  }

  if(digitalRead(39) == LOW){
    writeBit(package, 2, 1);
  }else{
    writeBit(package, 2, 0);
  }

  writeBit(package, 0, 1); 
  // writeBit(package, 1, digitalRead(ss_bit0)); 
  // writeBit(package, 2, digitalRead(ss_bit1)); 
  // writeBit(package, 3, digitalRead(ss_bit2)); 
  // writeBit(package, 4, digitalRead(ss_bit3)); 
  RTU_SLAVE.Hreg(1, package);
}



