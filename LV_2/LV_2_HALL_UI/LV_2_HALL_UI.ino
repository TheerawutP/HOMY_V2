#include <ModbusRTU.h>
#include <math.h>

#define SLAVE 1

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
#define SW_DW 25 //36
#define SW_UP 26 //39
#define SS 27 //34

ModbusRTU RTU_SLAVE;
//uint16_t lastSVal;
uint32_t time_print;
uint32_t last_time_print = 0;
uint32_t print_interval = 1000;

uint32_t last_time_up, last_time_down = 0;
uint32_t hold_state = 1000;

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

  pinMode(SW_DW, INPUT_PULLUP);   
  pinMode(SW_UP, INPUT_PULLUP);    
  pinMode(SS, INPUT_PULLUP);    

  pinMode(PB_DW_LAMP, OUTPUT);   
  pinMode(PB_UP_LAMP, OUTPUT);   
  pinMode(HALL_LCK, OUTPUT); 

  pinMode(DW_LAMP, OUTPUT);   
  pinMode(UP_LAMP, OUTPUT);    

  pinMode(seg_bit_0, OUTPUT);   
  pinMode(seg_bit_1, OUTPUT);   
  pinMode(seg_bit_2, OUTPUT);   
  pinMode(seg_bit_3, OUTPUT);   
 

}
  
void loop() {
  time_print = millis();
  uint16_t package;
  uint16_t up_frame;
  uint16_t dw_frame;
  uint16_t bit_r[16];
  
  RTU_SLAVE.task();
  ///////////////////////////////////////////////////////extract data from Hreg 0 (written by LV1)//////////////////////////////////////////

  uint16_t val = RTU_SLAVE.Hreg(0);
  //car motion dw/up
  bit_r[0] = (val & 0x0001) != 0;            
  bit_r[1] = (val & 0x0002) != 0; 
  //car pos in 4-bit
  bit_r[2] = (val & 0x0004) != 0;            
  bit_r[3] = (val & 0x0008) != 0;            
  bit_r[4] = (val & 0x0010) != 0;            
  bit_r[5] = (val & 0x0020) != 0;
  //confirm bits
  bit_r[6] = (val & 0x0040) != 0;            
  bit_r[7] = (val & 0x0080) != 0;            
  bit_r[8] = (val & 0x0100) != 0;            
  bit_r[9] = (val & 0x0200) != 0;            
  //hall lck/op/cl
  bit_r[10] = (val & 0x0400) != 0;            
  bit_r[11] = (val & 0x0800) != 0;            
  bit_r[12] = (val & 0x1000) != 0;   
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if(bit_r[10] == 1){
    digitalWrite(HALL_LCK, HIGH);
  }else{
    digitalWrite(HALL_LCK, LOW);
  }

  if(bit_r[11] == 1){
  }

  if(bit_r[12] == 1){
  }

  digitalWrite(DW_LAMP, bit_r[0]);
  digitalWrite(UP_LAMP, bit_r[1]);

  digitalWrite(seg_bit_0, bit_r[2]);
  digitalWrite(seg_bit_1, bit_r[3]);
  digitalWrite(seg_bit_2, bit_r[4]);
  digitalWrite(seg_bit_3, bit_r[5]);

///////////////////////////////////////////////// packing ////////////////////////////////////////////////////////////////////////////////////

  bool UP = digitalRead(SW_UP);
  bool DW = digitalRead(SW_DW);
  bool LCK = digitalRead(SS);

  if(LCK == 0){
    writeBit(package, 8, 1);
  }else{
    writeBit(package, 8, 0);
  }

  if(UP == 0){
    writeBit(package, 9, 1);
    last_time_up = millis();
  }else{
    if((millis() - last_time_up) >= hold_state){
      writeBit(package, 9, 0);      
    }
  }
  
  if(DW == 0){
    writeBit(package, 10, 1);
    last_time_down = millis();
  }else{
    if((millis() - last_time_down) >= hold_state){
          writeBit(package, 10, 0);
    }
  }
  

  RTU_SLAVE.Hreg(1, package);
  if((time_print - last_time_print) >= print_interval){
    last_time_print = time_print;
    Serial.println(" ");
    Serial.print("Hreg 0: "); Serial.println(RTU_SLAVE.Hreg(0));
    Serial.print("Hreg 1: "); Serial.println(RTU_SLAVE.Hreg(1));
    Serial.println(" ");
  }

}



