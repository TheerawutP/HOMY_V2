#include <ModbusRTU.h>
#include <math.h>


// #define ss_bit0 36
// #define ss_bit1 39
// #define ss_bit2 34
// #define ss_bit3 35

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

#define goto_f1 27
#define goto_f2 14
#define goto_f3 15

#define DW_LAMP(obj) digitalWrite(DOWN,obj)
#define UP_LAMP(obj) digitalWrite(UP,obj)
#define WAIT_IN_QUEUE(state, obj) digitalWrite(state ,obj)

QueueHandle_t xDisplayQueue;
TimerHandle_t xClearStateTimer;
ModbusRTU RTU_SLAVE;
uint16_t lastSVal;
uint16_t parsing_data[16];
uint16_t package;
unsigned long last = 0;
int h = 0;

int queuePins[6] = { out1, out2, out3, out4, out5, out6 };

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

void extractDataframe(uint16_t *frame, uint16_t val){
  for(int i = 0; i<=15; i++){
    frame[i] = (val >> i) & 0x0001;
  }
}

void pos_display(uint8_t floor){
  uint8_t bin[4] = {0};
  uint8_t i = 0;
    while (floor > 0) {
        bin[i] = floor % 2;
        floor = floor / 2;
        i++;
    }
  digitalWrite(segB0, bin[0]);
  digitalWrite(segB1, bin[1]);
  digitalWrite(segB2, bin[2]);
  digitalWrite(segB3, bin[3]);
}

// int encode_aiming(){
//   int bit0 = digitalRead(ss_bit0);
//   int bit1 = digitalRead(ss_bit1);
//   int bit2 = digitalRead(ss_bit2);
//   int bit3 = digitalRead(ss_bit3);
//   return (bit3 << 3) | (bit2 << 2) | (bit1 << 1) | bit0;
// }

void vModbusComTask(void *pvParameters){
  for(;;){
  RTU_SLAVE.task();
  uint16_t val = RTU_SLAVE.Hreg(0);
  extractDataframe(parsing_data, val);
  /*
  6 - 13 waiting in queue fl1 - 8 (turn on lamp fl1-8)

  5 car pos b3
  4 car pos b2
  3 car pos b1
  2 car pos b0

  1 on up lamp
  0 on dw lamp
  */
  xQueueSend(xDisplayQueue, parsing_data, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vAimTask(void *pvParameters){
  for(;;){
      // int target = encode_aiming();
      // if(target != 0){
      //   writeBit(package, 0, 1);                                  //dont forget to set 0 by STM after complete all cmd
      //   writeBit(package, target, 1);
      //   //xTimerStart(xHoldStateTimer, 0);
      //   RTU_SLAVE.Hreg(1, package);
      for(int i = 1; i<=8; i++){
        writeBit(package, i, parsing_data[i]);
      }

      int f1 = digitalRead(goto_f1);
      int f2 = digitalRead(goto_f2);
      int f3 = digitalRead(goto_f3);

        if(f1 == 0){
          writeBit(package, 0, 1);                                  
          writeBit(package, 1, 1);                                  
        }

        if(f2 == 0){
          writeBit(package, 0, 1);                                  
          writeBit(package, 2, 1);
        }

        if(f3 == 0){
          writeBit(package, 0, 1);                                  
          writeBit(package, 3, 1);
        }
      xTimerStart(xClearStateTimer, 0);
      RTU_SLAVE.Hreg(1, package);
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}
  
void vDisplayTask(void *pvParameters){
  uint16_t state[16];
  for(;;){

    if(xQueueReceive(xDisplayQueue, &state, portMAX_DELAY) == pdTRUE){
      
      DW_LAMP(state[0]);
      UP_LAMP(state[1]);
      
      uint8_t pos = (state[5] << 3) | (state[4] << 2) | (state[3] << 1) | state[2];
      pos_display(pos);

      for(int i = 0; i < 6; i++){
          WAIT_IN_QUEUE(queuePins[i], state[i + 6]);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vClearStateCallback(TimerHandle_t xTimer) {
          //active status bit
          writeBit(package, 0, 0);
          //aim to
          writeBit(package, 1, 0);
          writeBit(package, 2, 0);     
          writeBit(package, 3, 0); 

          RTU_SLAVE.Hreg(1, package);
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

  // pinMode(ss_bit0, INPUT);   
  // pinMode(ss_bit1, INPUT); 
  // pinMode(ss_bit2, INPUT);    
  // pinMode(ss_bit3, INPUT);   

  pinMode(goto_f1, INPUT_PULLUP);
  pinMode(goto_f2, INPUT_PULLUP);
  pinMode(goto_f3, INPUT_PULLUP);

  pinMode(segB0, OUTPUT);   
  pinMode(segB1, OUTPUT);   
  pinMode(segB2, OUTPUT);    
  pinMode(segB3, OUTPUT);    
  
  pinMode(DOWN, OUTPUT);    
  pinMode(UP, OUTPUT);  

  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(out4, OUTPUT);
  pinMode(out5, OUTPUT);
  pinMode(out6, OUTPUT);

  xClearStateTimer = xTimerCreate("Clear_State", pdMS_TO_TICKS(1000), pdFALSE, 0, vClearStateCallback);    

  xDisplayQueue = xQueueCreate(8, sizeof(parsing_data));  //handler for evnet queue
  xTaskCreate(vAimTask, "AimButtonHandle", 2048, NULL, 3, NULL);
  xTaskCreate(vModbusComTask, "ModbusCom", 2048, NULL, 3, NULL);
  xTaskCreate(vDisplayTask, "Display", 2048, NULL, 3, NULL);
}

void loop() {
  //       int f1 = digitalRead(goto_f1);
  //     int f2 = digitalRead(goto_f2);
  //     int f3 = digitalRead(goto_f3);
  // Serial.println(f1);
  // Serial.println(f2);
  // Serial.println(f3);
  //       vTaskDelay(pdMS_TO_TICKS(1000));


}



