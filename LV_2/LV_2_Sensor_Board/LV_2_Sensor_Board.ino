#include <ModbusRTU.h>
#include <math.h>

#define li_R 25  //light curtian
#define li_B 26
#define li_L 27

#define ss_op_R 33
#define ss_op_B 35   
#define ss_op_L 39    

#define ss_cl_R 32
#define ss_cl_B 34   
#define ss_cl_L 36    

#define lock_R 14
#define lock_B 12
#define lock_L 13

#define out_op_R  22
#define out_op_B  23
#define out_op_L  19

#define out_cl_R  18
#define out_cl_B  5
#define out_cl_L  4

#define LiF 23

ModbusRTU RTU_SLAVE;
uint16_t lastSVal;

unsigned long last = 0;
int h = 0;

uint16_t cbWrite(TRegister* reg, uint16_t val) {
  if (lastSVal != val) {
    lastSVal = val;
    Serial.println(String("HregSet val:") + String(val));
  }
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
  RTU_SLAVE.slave(1);        
  RTU_SLAVE.addHreg(0x0000,0);   //for write by main controller 
  RTU_SLAVE.onSetHreg(0x0000, cbWrite); //written enable   
  RTU_SLAVE.onGetHreg(0x0000, cbRead);
  
  RTU_SLAVE.addHreg(0x0001,0);   //for read by main controller 
  RTU_SLAVE.onSetHreg(0x0001, cbWrite); 
  RTU_SLAVE.onGetHreg(0x0001, cbRead); 

  pinMode(li_R, INPUT);
  pinMode(li_B, INPUT);
  pinMode(li_L, INPUT);

  pinMode(ss_op_R, INPUT);
  pinMode(ss_op_B, INPUT);
  pinMode(ss_op_L, INPUT);

  pinMode(ss_cl_R, INPUT);
  pinMode(ss_cl_B, INPUT);
  pinMode(ss_cl_L, INPUT);

  pinMode(lock_R, OUTPUT);
  pinMode(lock_B, OUTPUT);
  pinMode(lock_L, OUTPUT);

  pinMode(LiF, OUTPUT);
}

void loop() {
  uint16_t package;
  uint16_t bit_r[16];
  bool ready_L, ready_B, ready_R;

  RTU_SLAVE.task();
///////////////////////////////////////////////////extract data from Hreg 0 (written by LV1)//////////////////////////////////////

  uint16_t val = RTU_SLAVE.Hreg(0);
  //op cl L
  bit_r[0] = (val & 0x0001) != 0;            
  bit_r[1] = (val & 0x0002) != 0;  
  //op cl B
  bit_r[2] = (val & 0x0004) != 0;            
  bit_r[3] = (val & 0x0008) != 0;  
  //op cl R
  bit_r[4] = (val & 0x0010) != 0;            
  bit_r[5] = (val & 0x0020) != 0;
  //lock L B R
  bit_r[6] = (val & 0x0040) != 0;            
  bit_r[7] = (val & 0x0080) != 0;            
  bit_r[8] = (val & 0x0100) != 0; 
  // light and fan
  bit_r[9] = (val & 0x0200) != 0;    

//on-off Light and Fan
  if(bit_r[9]==1){
    digitalWrite(LiF, HIGH);
  }else {
    digitalWrite(LiF, LOW);
  }

//lock - unlock door L,B,R
  if(bit_r[6]==1){
    digitalWrite(lock_L, HIGH); ready_L = 1;
  }else{
    digitalWrite(lock_L, LOW); ready_L = 0;
  }

  if(bit_r[7]==1){
    digitalWrite(lock_B, HIGH); ready_B = 1;
  }else {
    digitalWrite(lock_B, LOW); ready_B = 0;
  }

  if(bit_r[8]==1){
    digitalWrite(lock_R, HIGH); ready_R = 1;
  }else {
    digitalWrite(lock_R, LOW); ready_R = 0;
  } 

//is door ready?
  if(digitalRead(li_L) == 1 && ready_L == 1){
    writeBit(package, 1, 1);
  }else{
    writeBit(package, 1, 0);
  }

  if(digitalRead(li_B) == 1 && ready_B == 1) {
    writeBit(package, 2, 1);
  }else{
    writeBit(package, 2, 0);
  }

  if(digitalRead(li_R) == 1 && ready_R == 1) {
      writeBit(package, 3, 1);
  }else{
      writeBit(package, 3, 0);
  }

//open-close door L
  if(bit_r[0] == 1 && bit_r[1] == 0){
    digitalWrite(out_cl_L, HIGH);
    writeBit(package, 4, 1);
    writeBit(package, 5, 0);

  }else if(bit_r[0] == 0 && bit_r[1] == 1){
    digitalWrite(out_op_L, HIGH);
    writeBit(package, 4, 0);
    writeBit(package, 5, 1);

  }else{
    digitalWrite(out_cl_L, LOW);
    digitalWrite(out_op_L, LOW);
    writeBit(package, 4, 0);
    writeBit(package, 5, 0);
  }

//open-close door B
  if(bit_r[2] == 1 && bit_r[3] == 0){
    digitalWrite(out_cl_B, HIGH);
    writeBit(package, 6, 1);
    writeBit(package, 7, 0);

  }else if(bit_r[2] == 0 && bit_r[3] == 1){
    digitalWrite(out_op_B, HIGH);
    writeBit(package, 6, 0);
    writeBit(package, 7, 1);

  }else{
    digitalWrite(out_cl_B, LOW);
    digitalWrite(out_op_B, LOW);
    writeBit(package, 6, 0);
    writeBit(package, 7, 0);
  }

//open-close door R
  if(bit_r[4] == 1 && bit_r[5] == 0){
    digitalWrite(out_cl_R, HIGH);
    writeBit(package, 8, 1);
    writeBit(package, 9, 0);

  }else if(bit_r[4] == 0 && bit_r[5] == 1){
    digitalWrite(out_op_R, HIGH);
    writeBit(package, 8, 0);
    writeBit(package, 9, 1);

  }else{
    digitalWrite(out_cl_R, LOW);
    digitalWrite(out_op_R, LOW);
    writeBit(package, 8, 0);
    writeBit(package, 9, 1);

  }

//bit alive
  writeBit(package, 0, 1);

  //RTU_SLAVE.Hreg(1,package);
  Serial.println(RTU_SLAVE.Hreg(0), BIN);


}



