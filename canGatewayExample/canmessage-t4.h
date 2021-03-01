#include <FlexCAN_T4.h>

uint8_t bitSize(uint64_t v) {
  static const uint8_t lookup[64] = {
  0, // change to 1 if you want bitSize(0) = 1
  1,  2, 53,  3,  7, 54, 27, 4, 38, 41,  8, 34, 55, 48, 28,
  62,  5, 39, 46, 44, 42, 22,  9, 24, 35, 59, 56, 49, 18, 29, 11,
  63, 52,  6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
  51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12
};
  static const uint64_t multiplicator = 0x022fdd63cc95386dUL;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  v++;

  return lookup[(uint64_t)(v * multiplicator) >> 58];
}

uint8_t reverse8(uint8_t inputData) {
  inputData = ((inputData * 0x0802LU & 0x22110LU) | (inputData * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16; 
  return inputData;
}

uint64_t reverse64(uint64_t inputData) {
  //reverse bytes
  //Efficient knuth 64 bit reverse
  static const uint64_t m0 = 0x5555555555555555LLU;
  static const uint64_t m1 = 0x0300c0303030c303LLU;
  static const uint64_t m2 = 0x00c0300c03f0003fLLU;
  static const uint64_t m3 = 0x00000ffc00003fffLLU;
  inputData = ((inputData >> 1) & m0) | (inputData & m0) << 1;
  inputData = inputData ^ (((inputData >> 4) ^ inputData) & m1) ^ ((((inputData >> 4) ^ inputData) & m1) << 4);
  inputData = inputData ^ (((inputData >> 8) ^ inputData) & m2) ^ ((((inputData >> 8) ^ inputData) & m2) << 8);
  inputData = inputData ^ (((inputData >> 20) ^ inputData) & m3) ^ ((((inputData >> 20) ^ inputData) & m3) << 20);
  inputData = (inputData >> 34) | (inputData << 30);
  return inputData;
}

void CAN_encode(CAN_message_t *msg, double inputDataDouble, uint8_t startBit, uint8_t bitLength, bool byteOrder, bool sign, double Scale, double bias) { //byteOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000LU;
  //scale and bias
  inputDataDouble = (1/Scale) * (inputDataDouble - bias);
  uint64_t maxVal = 1;
  for(int i=0; i<bitLength; i++) {
    maxVal *= 2;
  }
  //Sign value if appropriate
  if(sign) {
    if(inputDataDouble < 0) {
      inputDataDouble += maxVal;
    }
    inputData = inputDataDouble;
  }
  else
    inputData = inputDataDouble;

  if(inputData > (maxVal-1)) {
    inputData = (maxVal-1);
  }
  else if(inputData < 0){
    inputData = 0;
  }
  //access input as byte array and evaluate length of array
  //take advantage of endianness
  uint8_t *bytePointer = (uint8_t *)&inputData;

  if(byteOrder) {
    //locate MSB
    uint8_t trueLen = bitSize(inputData);
    //if more bits are present than can be accomodated cut them off
    if(trueLen > bitLength) {
      inputData = inputData >> (trueLen - bitLength);
    }
    //Shift buf to 64th position
    inputData = inputData << (64 - bitLength);

    //Reverse int
    inputData = reverse64(inputData);
    //calculate altered start position and move
    uint8_t calcStartbit = (7 - startBit%8) + 8*(startBit/8);
    inputData = inputData << calcStartbit;
    
    //reverse each byte and move to appropriate place
    for(int i=0; i<8; i++) {
      msg->buf[i] |= reverse8(bytePointer[i]);
    }
  }else {
    //Shift buf to appropriate place
    inputData = inputData << startBit;
    //push to message struct
    for(int i=0; i<8; i++) {
      msg->buf[i] |= bytePointer[i];
    }
  }
}

void CAN_encode(CAN_message_t *msg, int inputDataInt, uint8_t startBit, uint8_t bitLength, bool byteOrder, bool sign, double Scale, double bias) { //byteOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned

  uint64_t inputData = 0x0000000000000000LU;
  //scale and bias
  inputDataInt = (1/Scale) * (inputDataInt - bias);
  //Sign value if appropriate
  uint64_t maxVal = 1;
  for(int i=0; i<bitLength; i++) {
    maxVal *= 2;
  }
  
  if(sign) {

    if(inputDataInt < 0) {
      inputDataInt += maxVal;
    }
    inputData = inputDataInt;
  }
  else
    inputData = inputDataInt;

  if(inputData > (maxVal-1)) {
    inputData = (maxVal-1);
  }
  else if(inputData < 0){
    inputData = 0;
  }
  //access input as byte array
  //take advantage of endianness
  uint8_t *bytePointer = (uint8_t *)&inputData;

  if(byteOrder) {
    //locate MSB
    uint8_t trueLen = bitSize(inputData);
    //if more bits are present than can be accomodated cut them off
    if(trueLen > bitLength) {
      inputData = inputData >> (trueLen - bitLength);
    }
    //Shift buf to 64th position
    inputData = inputData << (64 - bitLength);

    //Reverse int
    inputData = reverse64(inputData);
    uint8_t calcStartbit = (7 - startBit%8) + 8*(startBit/8);
    inputData = inputData << calcStartbit;
    //reverse each byte and push to message struct
    for(int i=0; i<8; i++) {
      msg->buf[i] |= reverse8(bytePointer[i]);
    }
  }else {
    //Shift buf to appropriate place
    inputData = inputData << startBit;
    //push to message struct
    for(int i=0; i<8; i++) {
      msg->buf[i] |= bytePointer[i];
    }
  }
}


double CAN_decode(CAN_message_t *msg, uint8_t startBit, uint8_t bitLength, bool byteOrder, bool sign, double Scale, double bias) { //byteOrder is 1 for MSB, 0 for LSB. Signed is 1 for signed, 0 for unsigned
  //Create uint64
uint64_t dataOut;

  if(byteOrder) {
    uint8_t Bytes[8];
    
    //reverse all bytes
    for(int i=0; i<8; i++) {
      Bytes[i] = reverse8(msg->buf[i]);
    }
    //assemble to a uint64
    dataOut = *(uint64_t *)Bytes;
    //reverse to typical lsbfirst
    dataOut = reverse64(dataOut);
    //shift to isolate data
    uint8_t calcStartbit = (7 - (startBit%8)) + 8*(startBit/8);
    dataOut = dataOut << calcStartbit;
    //shift bits back
    dataOut = dataOut >> (64 - (bitLength));
  } else {
    //send message into uint64
    dataOut = *(uint64_t *)msg->buf;
    //Shift left then right to isolate the buf
    dataOut = dataOut << (64 - (startBit+bitLength));
    dataOut = dataOut >> (64 - bitLength);
  }
  double returnData = 0;
  //Adjust if signed and scale and bias
  if(sign) {
    uint64_t maxVal = 1;
    for(int i=0; i<bitLength; i++) {
      maxVal *= 2;
    }
    if(dataOut > (maxVal/2)) {
      returnData = (double) dataOut - maxVal;
      returnData = bias + (Scale * returnData);
    }
    else {
      returnData = bias + (Scale * dataOut);
    }
  }
  else {
    returnData = bias + (Scale * dataOut);
  }

  return(returnData);
}
