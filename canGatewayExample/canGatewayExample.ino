#include <FlexCAN_T4.h>
#include "canmessage-t4.h"

#define LSB 0
#define MSB 1
#define UNSIGNED 0
#define SIGNED 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

CAN_message_t outMsg;
CAN_message_t inMsg;

#define LEDpin 13
bool ledState = false;

//----Cyclic Timing Variables----////////////
elapsedMillis ms100Timer;
const int ms100 = 100;          //100 Milliseconds
/////////////////////////////////////////////

int counter = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("CAN Gateway example init...");

  pinMode(LEDpin, OUTPUT);
  
  can1.begin();
  can1.setBaudRate(500000);
  can2.begin();
  can2.setBaudRate(500000);
  can3.begin();
  can3.setBaudRate(500000);

  digitalWrite(LEDpin, HIGH);
  
  Serial.setTimeout(20);
  Serial.println("done");
}
void loop() {

  if (Serial.available() > 0){
    int tempint = Serial.parseInt();
    if (tempint > 0){
      Serial.print("Your input: ");
      Serial.println(tempint);
    }
  }

  if (ms100Timer >= ms100) {
    ms100Timer = ms100Timer - ms100;
    sendCanbusMessage1();
    sendCanbusMessage2();
    sendCanbusMessage3();
    
    if(counter < 256){counter++;}
    else{counter = 0;}
    ledState = !ledState;
    digitalWrite(LEDpin, ledState);
  }
  
  if (can1.read(inMsg)) {can1Read();}
  if (can2.read(inMsg)) {can2Read();}
  if (can3.read(inMsg)) {can3Read();}
}


void sendCanbusMessage1(){
  outMsg.id  = 0x001;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  CAN_encode(&outMsg, counter, 0, 8, LSB, UNSIGNED, 1, 0);
  can1.write(outMsg);
}

void sendCanbusMessage2(){
  outMsg.id  = 0x002;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  CAN_encode(&outMsg, counter, 0, 8, LSB, UNSIGNED, 1, 0);
  can2.write(outMsg);
}

void sendCanbusMessage3(){
  outMsg.id  = 0x003;
  outMsg.len = 8;
  outMsg.flags.extended = 0;
  for (int i = 0; i < outMsg.len; i++) {outMsg.buf[i] = 0x00;}
  CAN_encode(&outMsg, counter, 0, 8, LSB, UNSIGNED, 1, 0);
  can3.write(outMsg);
}

void can1Read(){
  can1.read(inMsg);
  switch (inMsg.id) { 
    case 0x004:
      int exampleDecodedvalue = CAN_decode(&inMsg, 0, 8, LSB, UNSIGNED, 1, 0);
      Serial.print("Example decoded value from CAN 1: ");
      Serial.println(exampleDecodedvalue);
    break;   
    default:
      break;
  }
}

void can2Read(){
  can2.read(inMsg);
  switch (inMsg.id) { 
    case 0x005:
      int exampleDecodedvalue = CAN_decode(&inMsg, 0, 8, LSB, UNSIGNED, 1, 0);
      Serial.print("Example decoded value from CAN 2: ");
      Serial.println(exampleDecodedvalue);
    break;   
    default:
      break;
  }
}

void can3Read(){
  can3.read(inMsg);
  switch (inMsg.id) { 
    case 0x006:
      int exampleDecodedvalue = CAN_decode(&inMsg, 0, 8, LSB, UNSIGNED, 1, 0);
      Serial.print("Example decoded value from CAN 3: ");
      Serial.println(exampleDecodedvalue);
    break;   
    default:
      break;
  }
}
