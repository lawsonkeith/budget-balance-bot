/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:11 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/Accel/adxl345.cpp,v $ 
 $Revision: 1.1 $
 $Log: adxl345.cpp,v $
 Revision 1.1  2014/04/28 20:46:11  keithl
 Latest revision

*/

#include "adxl345.h"
#include "i2c.h"
#include <Wire.h> 

ADXL::ADXL(char device) {
  _device = device;
}
void ADXL::init() {
  //the data format and range register is fine by default

    i2c.writeRegister(_device, ADXL_POWER_CTL, 0); //standby mode
  //we want the accelerometer to go to sleep when not in use
  i2c.writeRegister(_device, ADXL_DATA_FORMAT, 0b1011); //4 mg/LSB full scale
  i2c.writeRegister(_device, ADXL_THRESH_INACT, 16); //62.5 mg/LSB
  i2c.writeRegister(_device, ADXL_THRESH_ACT, 8); //62.5 mg/LSB
  i2c.writeRegister(_device, ADXL_TIME_INACT, 5); // 1s/LSB
  //i2c.writeRegister(_device, ADXL_BW_RATE, 0b1010); // 100Hz
	i2c.writeRegister(_device, ADXL_BW_RATE, 0b1100); // 400Hz

  //i2c.writeRegister(_device, ADXL_ACT_INACT_CTL, 0b11111111); // enable ac operation on all axes

  //i2c.writeRegister(_device, ADXL_INT_ENABLE, 0b10011000); // enable interrupts for activity/inactivity

  //now turn it on and start reading
  //i2c.writeRegister(_device, ADXL_POWER_CTL, (1<<5)|(1<<4)|(1<<3)); //link on, auto sleep on, measure on
  i2c.writeRegister(_device, ADXL_POWER_CTL, (1<<5)|(1<<3)); //link on, auto sleep OFF, measure on
  //calibrate();
}
int ADXL::readInt(char address) {
  Wire.beginTransmission(_device);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(_device,2);
  byte lsb=Wire.read();
  byte msb=Wire.read();
  return ((int)msb)<<8 | lsb;
}
int ADXL::getX() {
  return readInt(ADXL_DATAX0);
}
int ADXL::getY() {
  return readInt(ADXL_DATAY0);
}
int ADXL::getZ() {
  return readInt(ADXL_DATAZ0);
}

void ADXL::calibrate() {
  _offsetX=0, _offsetY=0, _offsetZ=0;
  long ox=0,oy=0,oz=0;

  char power=i2c.readRegister(_device, ADXL_POWER_CTL); //get the power setting
  i2c.writeRegister(_device, ADXL_POWER_CTL, 1<<3 ); //return to previous power mode

  //get some readings to generate offset values
  for (int i=0; i<ADXL_CALIBRATION_SAMPLES; i++) {
    while (i2c.readRegister(_device,ADXL_INT_SOURCE) & 0b10000000 == 0) {
    }
    ox+=readInt(ADXL_DATAX0);
    oy+=readInt(ADXL_DATAY0);
    oz+=readInt(ADXL_DATAZ0);
  }
  _offsetX=ox/ADXL_CALIBRATION_SAMPLES;
  _offsetY=oy/ADXL_CALIBRATION_SAMPLES;
  _offsetZ=oz/ADXL_CALIBRATION_SAMPLES;

  /*Serial.println("Offset");
  Serial.print(_offsetX);
  Serial.print('\t');
  Serial.print(_offsetY);
  Serial.print('\t');
  Serial.println(_offsetZ);*/

  i2c.writeRegister(_device, ADXL_POWER_CTL, power); //return to previous power mode
}

bool ADXL::dataReady() {
  char interrupt=i2c.readRegister(_device,ADXL_INT_SOURCE);
  return (interrupt & 0b10000000);
}
bool ADXL::sleeping() {
  char val=i2c.readRegister(_device,ADXL_ACT_TAP_STATUS);
  return (val & 0b1000);
}

