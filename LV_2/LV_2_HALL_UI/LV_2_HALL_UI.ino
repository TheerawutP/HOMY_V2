#include <ModbusRTU.h>
#include <math.h>
#define SLAVE 3

//OUTPUT
#define PB_DW_LAMP 23
#define PB_UP_LAMP 21
#define HALL_LCK 22

#define DW_LAMP 12
#define UP_LAMP 13

#define seg_bit_0 32
#define seg_bit_1 33
#define seg_bit_2 39 //25 
#define seg_bit_3 36 //26

//INPUT
#define SW_DW 19 //25 //36
#define SW_UP 18 //26 //39
#define SS 27 //34

TaskHandle_t xCallingButtonTaskHandle;
QueueHandle_t xProcessQueue;
TimerHandle_t xClearStateTimer;
typedef enum {
    DOOR_CLOSING,
    DOOR_OPENING,
    DOOR_CLOSED,
    DOOR_OPENED,
    DOOR_LOCK
}Door_Event_t;

typedef enum {
    CALL_UP,
    CALL_DW
}Button_Event_t;

///////////////////////////////////////////button class//////////////////////////////////////////

typedef enum {
    CALLING_UP,
    CALLING_DW,
    SPE
}BTN_TYPE;

typedef enum {
    IDLE,
    PRESSED,
    RELEASE
}BTN_STATE;

typedef struct {
    BTN_TYPE btn_type;
    BTN_STATE btn_state;
    uint8_t PIN;
}createButton;

typedef struct {
    createButton base;
    uint8_t floor_num;
}createButton_calling;

////////////////////////////////////////////////door class//////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------any objects here-----------------------------------
createButton_calling up_1 = {
    .base = {
        .btn_type = CALLING_UP, 
        .btn_state = IDLE, 
        .PIN = SW_UP
    },
    .floor_num = 1
};

createButton_calling dw_3 = {
    .base = {
        .btn_type = CALLING_DW, 
        .btn_state = IDLE, 
        .PIN = SW_DW
    },    
    .floor_num = 3
};

//----------------------------------------------------------------------------------------------------
ModbusRTU RTU_SLAVE;
//uint16_t lastSVal;
uint16_t package;
uint16_t parsing_data[16];

uint16_t cbWrite(TRegister* reg, uint16_t val) {
  // if (lastSVal != val) {
  //   lastSVal = val;
  //   Serial.println(String("HregSet val:") + String(val));
  // }
  return val;
}

uint16_t cbRead(TRegister* reg, uint16_t val) {
  reg->value = val;   
  return val;
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
    if (state) {
        value |= (1 << bit);     // set
    } else {
        value &= ~(1 << bit);    // clear
    }
}

void ISR_CALL_UP() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(
      xCallingButtonTaskHandle,
      SW_UP, 
      eSetValueWithOverwrite,
      &xHigherPriorityTaskWoken 
  );
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_CALL_DW() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(
      xCallingButtonTaskHandle,
      SW_DW, 
      eSetValueWithOverwrite,
      &xHigherPriorityTaskWoken 
  );
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vModbusComTask(void *pvParam){
  for(;;){
  RTU_SLAVE.task();
  uint16_t val = RTU_SLAVE.Hreg(0);

  //car motion dw/up
  parsing_data[0] = (val & 0x0001) != 0;            
  parsing_data[1] = (val & 0x0002) != 0; 
  //car pos in 4-bit
  parsing_data[2] = (val & 0x0004) != 0;            
  parsing_data[3] = (val & 0x0008) != 0;            
  parsing_data[4] = (val & 0x0010) != 0;            
  parsing_data[5] = (val & 0x0020) != 0;
  //confirm bits
  parsing_data[6] = (val & 0x0040) != 0;            
  parsing_data[7] = (val & 0x0080) != 0;            
  parsing_data[8] = (val & 0x0100) != 0;            
  parsing_data[9] = (val & 0x0200) != 0;            
  //hall lck/op/cl
  parsing_data[10] = (val & 0x0400) != 0;            
  parsing_data[11] = (val & 0x0800) != 0;            
  parsing_data[12] = (val & 0x1000) != 0;   
  xQueueSend(xProcessQueue, parsing_data, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vCallingButtonTask(void *pvParam){
    uint32_t val;
    for (;;) {
        if (xTaskNotifyWait(0, 0, &val, portMAX_DELAY) == pdTRUE) {
            
            // switch (val) {
            //   case SW_UP:
            //     if(digitalRead(SW_UP) == LOW){
            //       writeBit(package, 9, 1);
            //       xTimerStart(xHoldButtonTimer, 0);
            //     }
            //     RTU_SLAVE.Hreg(1, package);
            //     break;

              
            //   case SW_DW:
            //     if(digitalRead(SW_DW) == LOW) {
            //       writeBit(package, 10, 1);
            //       xTimerStart(xHoldButtonTimer, 0);
            //     }
            //     RTU_SLAVE.Hreg(1, package);
            //     break;


            // if (val == SW_UP) {
            //     // Keep setting bit as long as button is pressed
            //     while(digitalRead(SW_UP) == LOW) {
            //         writeBit(package, 9, 1);
            //         RTU_SLAVE.Hreg(1, package);
            //         vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
            //     }
            //     // Button released, clear bit
            //     writeBit(package, 9, 0);
            //     RTU_SLAVE.Hreg(1, package);
            // }
            
            // if (val == SW_DW) {
            //     // Keep setting bit as long as button is pressed
            //     while(digitalRead(SW_DW) == LOW) {
            //         writeBit(package, 10, 1);
            //         RTU_SLAVE.Hreg(1, package);
            //         vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
            //     }
            //     // Button released, clear bit
            //     writeBit(package, 10, 0);
            //     RTU_SLAVE.Hreg(1, package);
            // }


            if(val == SW_UP){
                writeBit(package, 9, 1);
                RTU_SLAVE.Hreg(1, package);   
                if(digitalRead(SW_UP) == LOW) digitalWrite(DW_LAMP, HIGH);
                xTimerStart(xClearStateTimer, 0);        
            }

            if(val == SW_DW){
                writeBit(package, 10, 1);
                RTU_SLAVE.Hreg(1, package);                                  
                if(digitalRead(SW_DW) == LOW) digitalWrite(DW_LAMP, HIGH);
                xTimerStart(xClearStateTimer, 0);               
            }


        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void vDoorTask(){
//   if(LCK == 0){
//     writeBit(package, 8, 1);
//   }else{
//     writeBit(package, 8, 0);
//   }
// }

void vClearStateCallback(TimerHandle_t xTimer) {
    // if(digitalRead(SW_UP) == HIGH){
    //   writeBit(package, 9, 0);      
    // }
    // if(digitalRead(SW_DW) == HIGH){
    //   writeBit(package, 10, 0);
    // }
    digitalWrite(PB_UP_LAMP, LOW);
    digitalWrite(PB_DW_LAMP, LOW);
    writeBit(package, 9, 0);
    writeBit(package, 10, 0);
    RTU_SLAVE.Hreg(1, package);
}


void vProcessTask(void *pvParam){
  uint16_t frame[16];
  for(;;){
      if (xQueueReceive(xProcessQueue, &frame, portMAX_DELAY) == pdTRUE) {
          if(frame[10] == 1){
             digitalWrite(HALL_LCK, HIGH);
          }else{
             digitalWrite(HALL_LCK, LOW);
          }
       // if(parsing_data[11] == 1){
       // }
       // if(parsing_data[12] == 1){
       // }
          digitalWrite(DW_LAMP, frame[0]);
          digitalWrite(UP_LAMP, frame[1]);

          // digitalWrite(seg_bit_0, frame[2]);
          // digitalWrite(seg_bit_1, frame[3]);
          // digitalWrite(seg_bit_2, frame[4]);
          // digitalWrite(seg_bit_3, frame[5]);
      }
      
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8E1, 16, 17); // RX=16, TX=17
  
  RTU_SLAVE.begin(&Serial2);
  RTU_SLAVE.slave(SLAVE);      

  RTU_SLAVE.addHreg(0x0000,0);   
  RTU_SLAVE.onSetHreg(0x0000, cbWrite); 
  RTU_SLAVE.onGetHreg(0x0000, cbRead);
  
  RTU_SLAVE.addHreg(0x0001,0);   
  RTU_SLAVE.onSetHreg(0x0001, cbWrite); 
  RTU_SLAVE.onGetHreg(0x0001, cbRead);

  //calling up&dw
  pinMode(SW_DW, INPUT_PULLUP);   
  attachInterrupt(digitalPinToInterrupt(SW_DW), ISR_CALL_DW, FALLING);


  pinMode(SW_UP, INPUT_PULLUP);    
  attachInterrupt(digitalPinToInterrupt(SW_UP), ISR_CALL_UP, FALLING);

  pinMode(SS, INPUT_PULLUP);    

  pinMode(PB_DW_LAMP, OUTPUT);   
  pinMode(PB_UP_LAMP, OUTPUT);   
  pinMode(HALL_LCK, OUTPUT); 

  pinMode(DW_LAMP, OUTPUT);   
  pinMode(UP_LAMP, OUTPUT);    

  // pinMode(seg_bit_0, OUTPUT);   
  // pinMode(seg_bit_1, OUTPUT);   
  // pinMode(seg_bit_2, OUTPUT);   
  // pinMode(seg_bit_3, OUTPUT);   
  
  xProcessQueue = xQueueCreate(10, sizeof(parsing_data));
  xTaskCreate(vProcessTask, "Processing", 1024, NULL, 3, NULL);
  xTaskCreate(vModbusComTask, "ModbusCom", 2048, NULL, 3, NULL);
  xClearStateTimer = xTimerCreate("Clear_State", pdMS_TO_TICKS(1500), pdFALSE, 0, vClearStateCallback);

  xTaskCreate(
    vCallingButtonTask,         
    "Calling",        
    1024,                 
    NULL,                
    3,                  
    &xCallingButtonTaskHandle
  );

}
  
void loop() {

}



