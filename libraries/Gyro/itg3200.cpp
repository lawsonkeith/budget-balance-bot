/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:50 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/Gyro/itg3200.cpp,v $ 
 $Revision: 1.1 $
 $Log: itg3200.cpp,v $
 Revision 1.1  2014/04/28 20:46:50  keithl
 Latest revision

*/
#include "itg3200.h"
#include "i2c.h"
#include <Wire.h> 

//#define ITG_PWR_MODE 0b00001011 //use z axis oscillator as clock source, z in standby
//#define ITG_PWR_MODE 0b00000011 //use z axis oscillator as clock source
#define ITG_PWR_MODE 0b00000000 //use internal oscillator as clock source
#define ITG_CALIBRATION_SAMPLES 512 //number of samples to average to work out the offset

ITG::ITG(char device) 
{
  _device=device;
}




void ITG::init() {
  i2c.writeRegister(_device, ITG_PWR_MGM, 1<<7); //reset the device
 // i2c.writeRegister(_device, ITG_SMPLRT_DIV, 9); //sample rate divider to give 100Hz
i2c.writeRegister(_device, ITG_SMPLRT_DIV, 1); //sample rate divider to give 500Hz
  i2c.writeRegister(_device, ITG_DLPF_FS, 0b00011011); //42Hz low pass, 1kHz internal sample rate
  i2c.writeRegister(_device, ITG_INT_CFG, 0b100101); //interrupt for data available and new clock
  i2c.writeRegister(_device, ITG_PWR_MGM, ITG_PWR_MODE); 
  delay(70);
  /*while ((i2c.readRegister(_device, ITG_INT_STATUS) & 0b100) ==0 )
  {
    Serial.println("Waiting for clock sync.");
  }*/
  calibrate();
  sleep(); // start the device in sleep mode
}




int ITG::readInt(char address) 
{
  Wire.beginTransmission(_device);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(_device,2);
  byte msb=Wire.read();
  byte lsb=Wire.read();
  return ((int)msb)<<8 | lsb;
}



float ITG::getX() 
{
  return ((float)readInt(ITG_GYRO_XOUT_H) - _offsetX)/14.375;
}



float ITG::getY() 
{
  return ((float)readInt(ITG_GYRO_YOUT_H)-_offsetY)/14.375;
}


float ITG::getZ() 
{
  return ((float)readInt(ITG_GYRO_ZOUT_H)-_offsetZ)/14.375;
}


float ITG::getTemp() 
{
  //return (float)readInt(ITG_GYRO_TEMP_OUT_H);
  return ((float)readInt(ITG_GYRO_TEMP_OUT_H)+13200)/280.0+35.0;
}


void ITG::sleep() 
{
  char power = i2c.readRegister(_device, ITG_PWR_MGM);
  power = power | 0b01000000; //turn on the sleep bit
  i2c.writeRegister(_device, ITG_PWR_MGM, power);
}


void ITG::wake() {
  char power = i2c.readRegister(_device, ITG_PWR_MGM);
  if (power & 0b01000000 ==0)
    return;
  power = power & ~0b01000000; //turn off the sleep bit
  i2c.writeRegister(_device, ITG_PWR_MGM, power);
  delay(70);
}



void ITG::calibrate() 
{
  //delay(2000);
  _offsetX=0, _offsetY=0, _offsetZ=0;
  long ox=0,oy=0,oz=0;

  char power = i2c.readRegister(_device, ITG_PWR_MGM);
  char sample = i2c.readRegister(_device, ITG_SMPLRT_DIV);
  //wake(); //in case we were asleep
  //i2c::writeRegister(_device, ITG_SMPLRT_DIV, 0); //max sample rate
  
    //get some readings to generate offset values
  for (int i=0; i<ITG_CALIBRATION_SAMPLES; i++) {
    while (!dataReady()) {
    }
    ox+=readInt(ITG_GYRO_XOUT_H);
    oy+=readInt(ITG_GYRO_YOUT_H);
    oz+=readInt(ITG_GYRO_ZOUT_H);
  }
  _offsetX=ox/ITG_CALIBRATION_SAMPLES;
  _offsetY=oy/ITG_CALIBRATION_SAMPLES;
  _offsetZ=oz/ITG_CALIBRATION_SAMPLES;
  
  /*Serial.println("Offset");
  Serial.print(_offsetX);
  Serial.print('\t');
  Serial.print(_offsetY);
  Serial.print('\t');
  Serial.println(_offsetZ);*/

  //i2c::writeRegister(_device, ITG_SMPLRT_DIV, sample); //bring the sample rate back to what it was
  i2c.writeRegister(_device, ITG_PWR_MGM, power); //return to previous power mode
  //delay(2000);
}


bool ITG::dataReady() 
{
  Wire.beginTransmission(_device);
  Wire.write(ITG_INT_STATUS);
  Wire.endTransmission();
  Wire.requestFrom(_device,1);
  char interrupt=Wire.read();
  //Serial.println(interrupt, BIN);
  return (interrupt & 0b00000001);
}

