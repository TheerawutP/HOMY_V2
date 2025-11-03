#include <ModbusRTU.h>
#include <math.h>
#include <stdio.h>

//output
#define EMER_L 26
#define BELL_L 14
#define OP_L 19
#define CL_L 18
#define HLD_L 5
#define EN 23
#define FR 22
//#define SPD     //on speed 2 

//input
#define pin_open 32
#define pin_close 33  
#define pin_hold 25  
#define pin_bell 36
#define pin_emer 39
//#define landing

//evnet types 
typedef enum {
    BTN_OPEN,
    BTN_CLOSE,
    BTN_HOLD,
    BTN_RELEASE_HOLD,
    BTN_EMERGENCY,
    BTN_BELL,
    BTN_LANDING,
    FULL_OPEN,
    FULL_CLOSE,
    OBSTACLE
} ButtonEvent_t;

//door' states
typedef enum { 
  DOOR_CLOSE, 
  DOOR_OPEN,
  DOOR_HOLD,
  DOOR_IDLE 
}Type_state_t;
//DoorState_t doorState = DOOR_IDLE; //better check from sensor

typedef struct{
  Type_state_t pre_state;
  Type_state_t now_state;
}DoorState_t;

DoorState_t doorState = {DOOR_IDLE, DOOR_IDLE};


QueueHandle_t xButtonQueue;
//QueueHandle_t xModbusComQueue;
TimerHandle_t xLastStateTimer;

ModbusRTU RTU_SLAVE;
uint16_t parsing_data[16];
uint16_t package;
uint16_t lastSVal;

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

//change state at specific bit in uint16_t val 
void writeBit(uint16_t &value, uint8_t bit, bool state) {
    if (state) {
        value |= (1 << bit);     // set
    } else {
        value &= ~(1 << bit);    // clear
    }
}

void Door_Open(){
   digitalWrite(EN, HIGH);
   digitalWrite(FR, HIGH);
}

void Door_Close(){
   digitalWrite(EN, HIGH);
   digitalWrite(FR, LOW);
}

void Door_Stay(){
   digitalWrite(EN, LOW);
}

//extract bit 0-15 from Written Hreg into array 
void vModbusComTask(void *pvParameters){
  for(;;){
  RTU_SLAVE.task();
  uint16_t val = RTU_SLAVE.Hreg(0);
    //op cl L
  parsing_data[0] = (val & 0x0001) != 0;            
  parsing_data[1] = (val & 0x0002) != 0;  
  //op cl B
  parsing_data[2] = (val & 0x0004) != 0;            
  parsing_data[3] = (val & 0x0008) != 0;  
  //op cl R
  parsing_data[4] = (val & 0x0010) != 0;            
  parsing_data[5] = (val & 0x0020) != 0;
  //light curtian
  parsing_data[6] = (val & 0x0040) != 0;            
  parsing_data[7] = (val & 0x0080) != 0;            
  parsing_data[8] = (val & 0x0100) != 0; 

//if some event occurs, send that type of event to handler 
  if(parsing_data[0] == 1){
    ButtonEvent_t evt = FULL_OPEN;
    xQueueSend(xButtonQueue, &evt, 0);
  }
  if(parsing_data[1] == 1){
    ButtonEvent_t evt = FULL_CLOSE;
    xQueueSend(xButtonQueue, &evt, 0);
  }
  if(parsing_data[8] == 1){
    ButtonEvent_t evt = OBSTACLE;
    xQueueSend(xButtonQueue, &evt, 0);
  }

//packing data
  RTU_SLAVE.Hreg(1, package);
  package = 0;

//yield for 10 ticks, let other task works
  vTaskDelay(pdMS_TO_TICKS(10));
  }
  //xQueueSend(xModbusQueue, &parsing_data, portMAX_DELAY);
}

void vDoorControlTask(void *pvParameters) {
    ButtonEvent_t event;
    for (;;) {
        if (xQueueReceive(xButtonQueue, &event, portMAX_DELAY) == pdTRUE) {
            switch (event) {

                case BTN_OPEN:
                    if(doorState.now_state != DOOR_OPEN){
                      //remember door's last state
                      doorState.pre_state = doorState.now_state;
                      doorState.now_state = DOOR_OPEN;
                      //write event type into dataframe
                      writeBit(package, 1, 1);
                      Door_Open();
                    }
                    break;

                case BTN_CLOSE:
                    if(doorState.now_state != DOOR_CLOSE){
                      doorState.pre_state = doorState.now_state;
                      doorState.now_state = DOOR_CLOSE;
                      writeBit(package, 2, 1);
                      Door_Close();
                    }
                    break;

                case BTN_HOLD:
                    //if the current state isnt hold, then force door driver to off
                    if(doorState.now_state == DOOR_OPEN || doorState.now_state == DOOR_CLOSE){
                      doorState.pre_state = doorState.now_state;
                      doorState.now_state = DOOR_HOLD;
                      writeBit(package, 3, 1);
                      Door_Stay();
                    }
                    break;

                case BTN_RELEASE_HOLD:
                    //after release hold button for X ms, the state of door goes back to its last state then that handler of state start operates again
                    if (doorState.now_state == DOOR_HOLD) {
                      xTimerStart(xLastStateTimer, 0);
                      Door_Stay();
                    }
                    break;

                case BTN_BELL:
                    break;

                case BTN_EMERGENCY:
                    break;

                case BTN_LANDING:
                    break;

                case FULL_OPEN:
                    doorState.pre_state = doorState.now_state;
                    doorState.now_state = DOOR_IDLE;                   
                    Door_Stay();
                    break;

                case FULL_CLOSE:
                    doorState.pre_state = doorState.now_state;
                    doorState.now_state = DOOR_IDLE;
                    Door_Stay();
                    break;

                case OBSTACLE:
                    doorState.pre_state = doorState.now_state;
                    doorState.now_state = DOOR_IDLE;
                    Door_Stay();
                    break;           
            }
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vLastStateCallback(TimerHandle_t xTimer) {
  doorState.now_state = doorState.pre_state;
}

void ISR_Open() {
  //False is dont let higher priority task takes cpu from this ISR
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_OPEN;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Hold() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_HOLD;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Release_Hold() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_RELEASE_HOLD;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Close() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_CLOSE;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Emergency() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_EMERGENCY;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Bell() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_BELL;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_SafteyLanding() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  ButtonEvent_t evt = BTN_LANDING;
  xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// void ISR_Full_Closed() {
//   if(parsing_data[1] == 1){
//       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//       ButtonEvent_t evt = FULL_CLOSE;
//       xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
//       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//   }
// }

// void ISR_Full_Opened() {
//   if(parsing_data[0] == 1){
//       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//       ButtonEvent_t evt = FULL_OPEN;
//       xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
//       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//   }
// }

// void ISR_LightCurtian() {
//   if(parsing_data[8] == 1){
//       BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//       ButtonEvent_t evt = OBSTACLE;
//       xQueueSendFromISR(xButtonQueue, &evt, &xHigherPriorityTaskWoken);
//       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//   }




void setup() {
  Serial.begin(115200);
  //setup as master
  Serial2.begin(38400, SERIAL_8E1, 16, 17); // RX=16, TX=17

  //setup as slave 
  RTU_SLAVE.begin(&Serial2);
  RTU_SLAVE.slave(3);        
  
  //initiate Hreg
  RTU_SLAVE.addHreg(0x0000,0);   //for write by main controller 
  RTU_SLAVE.onSetHreg(0x0000, cbWrite); //written enable   
  RTU_SLAVE.onGetHreg(0x0000, cbRead);
  
  RTU_SLAVE.addHreg(0x0001,0);   //for read by main controller 
  RTU_SLAVE.onSetHreg(0x0001, cbWrite); 
  RTU_SLAVE.onGetHreg(0x0001, cbRead); 

  //attached EXTI to GPIO
  pinMode(pin_open, INPUT_PULLUP);   //INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(pin_open), ISR_Open, FALLING);

  pinMode(pin_close, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_close), ISR_Close, FALLING);

  pinMode(pin_hold, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_hold), ISR_Hold, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_hold), ISR_Release_Hold, RISING);

  pinMode(pin_bell, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_bell), ISR_Bell, FALLING);

  pinMode(pin_emer, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_emer), ISR_Emergency, FALLING);

  //pinMode(landing, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(landing), ISR_Landing, FALLING);

  pinMode(EMER_L, OUTPUT);
  pinMode(BELL_L, OUTPUT);
  pinMode(OP_L, OUTPUT);
  pinMode(CL_L, OUTPUT);
  pinMode(HLD_L, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(FR, OUTPUT);

  xButtonQueue = xQueueCreate(10, sizeof(ButtonEvent_t));  //handler for evnet queue
  //xModbusQueue = xQueueCreate(20, sizeof()); //16 = uint16_t     
  xLastStateTimer = xTimerCreate("LastState", pdMS_TO_TICKS(1500), pdFALSE, 0, vLastStateCallback);    
  xTaskCreate(vDoorControlTask, "DoorControl", 2048, NULL, 3, NULL);
  xTaskCreate(vModbusComTask, "ModbusCom", 2048, NULL, 3, NULL);

}

void loop() {          
}



