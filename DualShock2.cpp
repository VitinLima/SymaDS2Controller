#include "DualShock2.h"
#include "arduino.h"

#define SPI_SPEED 100000
#define SPI_DELAY_us 100

DualShock2::DualShock2(int _CS){
  CS = _CS;
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
}

void DualShock2::configurate(bool ANALOG_MODE, bool ANALOG_LOCKED, bool PRESSURE_MODE){
  commandHeader[1] = 0x43;
  fillCommandData(0x0000);
  commandMainTransfer[0] = 0x0100;
  transfer();

  commandHeader[1] = 0x44;
  fillCommandData(0x0000);
  if(ANALOG_MODE){
    commandMainTransfer[0] |= 0x0100;
  }
  if(ANALOG_LOCKED){
    commandMainTransfer[0] |= 0x0003;
  }
  transfer();

  commandHeader[1] = 0x4f;
  fillCommandData(0x00);
  if(PRESSURE_MODE){
    commandMainTransfer[0] = 0xffff;
    commandMainTransfer[1] = 0x0300;
  }
  transfer();
  
  commandHeader[1] = 0x43;
  fillCommandData(0x0000);
  transfer();
  
  commandHeader[1] = 0x42;
  fillCommandData(0x0000);
  transfer();
}

void DualShock2::update(){
  commandHeader[1] = 0x42;
  fillCommandData(0x0000);
  commandMainTransfer[0] = commandMotors;
  transfer();
}

bool DualShock2::isConnected(){
  return _isConnected;
}

void DualShock2::transfer(){
  SPI.beginTransaction(SPISettings(SPI_SPEED, LSBFIRST, SPI_MODE3));
  digitalWrite(CS, LOW);
  readControllerHeader();
  readControllerData();
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void DualShock2::readControllerHeader(){
  dataHeader[0] = SPI.transfer(commandHeader[0]);
  delayMicroseconds(SPI_DELAY_us);
  dataHeader[1] = SPI.transfer(commandHeader[1]);
  delayMicroseconds(SPI_DELAY_us);
  dataHeader[2] = SPI.transfer(commandHeader[2]);
  delayMicroseconds(SPI_DELAY_us);
  if(dataHeader[1] == 0x41 || dataHeader[1] == 0x73 || dataHeader[1] == 0x79 || dataHeader[1] == 0xF3){
    _isConnected = true;
    controllerMode = (dataHeader[1]&0xf0)>>4;
    mainTransferLength = dataHeader[1]&0x0f;
    if(mainTransferLength>9){
      mainTransferLength=9;
    }
  } else{
    _isConnected = false;
    controllerMode = 0;
    mainTransferLength = 0;
    dataHeader[1] &= 0xF0;
  }
}

void DualShock2::readControllerData(){
  uint8_t outByte;
  uint8_t inByte;
  for(int i = 0; i < mainTransferLength; i++){
    outByte = commandMainTransfer[i]>>8;
    inByte = SPI.transfer(outByte);
    dataMainTransfer[i] = inByte<<8;
    delayMicroseconds(SPI_DELAY_us);
    outByte = commandMainTransfer[i]&0x00FF;
    inByte = SPI.transfer(outByte);
    dataMainTransfer[i] |= inByte;
    delayMicroseconds(SPI_DELAY_us);
  }
}

bool DualShock2::getButtonState(uint16_t button){
  return button&dataMainTransfer[0] ? false : true;
}

uint8_t DualShock2::getButtonPressure(uint16_t button){
  if(controllerMode != 7){
    return uint8_t(0x00);
  }
  switch(button){
      case PAD_UP:
        return uint8_t((dataMainTransfer[3]>>8)&0x00ff);
        break;
      case PAD_LEFT:
        return uint8_t(dataMainTransfer[3]&0x00ff);
        break;
      case PAD_DOWN:
        return uint8_t((dataMainTransfer[4]>>8)&0x00ff);
        break;
      case PAD_RIGHT:
        return uint8_t(dataMainTransfer[4]&0x00ff);
        break;
      case PAD_L2:
        return uint8_t((dataMainTransfer[5]>>8)&0x00ff);
        break;
      case PAD_R2:
        return uint8_t(dataMainTransfer[5]&0x00ff);
        break;
      case PAD_L1:
        return uint8_t((dataMainTransfer[6]>>8)&0x00ff);
        break;
      case PAD_R1:
        return uint8_t(dataMainTransfer[6]&0x00ff);
        break;
      case PAD_TRIANGLE:
        return uint8_t((dataMainTransfer[7]>>8)&0x00ff);
        break;
      case PAD_O:
        return uint8_t(dataMainTransfer[7]&0x00ff);
        break;
      case PAD_X:
        return uint8_t((dataMainTransfer[8]>>8)&0x00ff);
        break;
      case PAD_SQUARE:
        return uint8_t(dataMainTransfer[8]&0x00ff);
        break;
    default:
      return uint8_t(0x00);
  }
}

uint8_t DualShock2::getJoystick(uint32_t joystick){
  if(controllerMode != 7){
    return uint8_t(0x80);
  }
  switch(joystick){
    case JOYSTICK_Ry:
      return joystick&dataMainTransfer[1];
      break;
    case JOYSTICK_Rx:
      return (joystick&dataMainTransfer[1])>>8;
      break;
    case JOYSTICK_Ly:
      return (joystick>>16)&dataMainTransfer[2];
      break;
    case JOYSTICK_Lx:
      return ((joystick>>16)&dataMainTransfer[2])>>8;
      break;
  }
}

void DualShock2::setLargeMotor(uint8_t LMotorValue){
  if(motorMap_1){
    commandMotors = (commandMotors&0x00ff) | uint16_t(LMotorValue)<<8;
  }else if(motorMap_1){
    commandMotors = (commandMotors&0xff00) | uint16_t(LMotorValue);
  }
}

void DualShock2::setSmallMotor(uint8_t SMotorValue){
  if(!motorMap_1){
    commandMotors = (commandMotors&0x00ff) | uint16_t(SMotorValue)<<8;
  }else if(!motorMap_2){
    commandMotors = (commandMotors&0xff00) | uint16_t(SMotorValue);
  }
}

bool DualShock2::isAnalog(){
  return controllerMode == 7;
}

bool DualShock2::isDigital(){
  return controllerMode == 4;
}

void DualShock2::fillCommandData(uint16_t fillingWord){
  for(int i = 0; i < 9; i++){
    commandMainTransfer[i] = fillingWord;
  }
}

void DualShock2::getData(uint8_t *buf){
  for(uint8_t i = 0; i<3; i++){
    buf[i] = dataHeader[i];
  }
  for(uint8_t i = 0; i<mainTransferLength; i++){
    buf[3+2*i] = dataMainTransfer[i]>>8;
    buf[4+2*i] = dataMainTransfer[i]&0x00FF;
  }
}

void DualShock2::buttonsInPolling(){
  commandHeader[1] = 0x41;
  fillCommandData(0x5A5A);
  transfer();
}

void DualShock2::mainPolling(){
  commandHeader[1] = 0x42;
  fillCommandData(0x0000);
  commandMainTransfer[0] = commandMotors;
  transfer();
}

void DualShock2::setConfigMode(bool CONFIG_MODE){
  commandHeader[1] = 0x43;
  fillCommandData(0x0000);
  commandMainTransfer[0] = CONFIG_MODE ? 0x0100 : 0x0000;
  transfer();
}

void DualShock2::setAnalogMode(bool ANALOG_MODE, bool ANALOG_LOCKED){
  commandHeader[1] = 0x44;
  fillCommandData(0x0000);
  commandMainTransfer[0] |= ANALOG_MODE ? 0x0100 : 0x0000;
  commandMainTransfer[0] |= ANALOG_LOCKED ? 0x0003 : 0x0000;
  transfer();
}

void DualShock2::getMoreStatusInfo(){
  commandHeader[1] = 0x45;
  fillCommandData(0x5A5A);
  transfer();
}

void DualShock2::mapMotors(bool motor_1, bool motor_2){
  commandHeader[1] = 0x4D;
  fillCommandData(0xFF);
  commandMainTransfer[0] = motor_1 ? 0x0100 : 0x0000;
  commandMainTransfer[0] |= motor_2 ? 0x0001 : 0x0000;
  motorMap_1 = motor_1;
  motorMap_2 = motor_2;
  transfer();
}

void DualShock2::setAnalogResponses(uint8_t b1, uint8_t b2, uint8_t b3){
  commandHeader[1] = 0x4F;
  b3 &= 0x03;
  commandMainTransfer[0] = b1;
  commandMainTransfer[1] = b2;
  commandMainTransfer[2] = b3;
}

/*void DualShock2::runCommand(uint8_t command){
  commandHeader[1] = command;
  switch(commandHeader[1]){
    case 0x41:
      fillCommandData(0x5a5a);
      break;
      
    case 0x42:
      fillCommandData(0x0000);
      commandMainTransfer[0] = commandMotors;
      break;
      
    case 0x43:
      fillCommandData(0x0000);
      break;
      
    case 0x44:
      fillCommandData(0x0000);
      break;
      
    case 0x45:
      fillCommandData(0x5a5a);
      break;
      
    case 0x46:
      fillCommandData(0x5a5a);
      commandMainTransfer[0] = 0x00;
      transfer();
      
      fillCommandData(0x5a5a);
      commandMainTransfer[0] = 0x01;
      transfer();
      
      commandHeader[1] = 0x47;
      fillCommandData(0x5a5a);
      commandMainTransfer[0] = 0x00;
      transfer();
      
      commandHeader[1] = 0x4c;
      fillCommandData(0x5a5a);
      commandMainTransfer[0] = 0x00;
      transfer();
      
      fillCommandData(0x5a5a);
      commandMainTransfer[0] = 0x01;
      transfer();
      
      commandHeader[1] = 0x42;
      break;
      
    case 0x4d:
      fillCommandData(0xffff);
      commandHeader[1] = 0x4d;
      break;
      
    case 0x4f:
      fillCommandData(0x0000);
      commandHeader[1] = 0x4f;
      break;
      
    default:
      fillCommandData(0x0000);
      commandMainTransfer[0] = commandMotors;
      commandHeader[1] = 0x42;
  }
  transfer();
}*/

//uint8_t DualShock2::getDataMainTransferByte(int idx){
//  if(idx>8 || idx < 0){
//    return 0x00;
//  }
//  return dataMainTransfer[idx];
//}
