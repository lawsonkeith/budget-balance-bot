/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:34 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/i2c/i2c.h,v $ 
 $Revision: 1.1 $
 $Log: i2c.h,v $
 Revision 1.1  2014/04/28 20:46:34  keithl
 Latest revision

*/
//i2c.h
//defines the constants for i2c operation
#ifndef i2c_h
#define i2c_h

#include "Arduino.h"

#define CLOCK_RATE 8000000
#define I2C_BAUD ((CLOCK_RATE/400000)-16)/2 //400kHz fast mode I2C

#define START 0x08
#define RESTART 0x10
#define MR_SLA_ACK 0x40
#define MR_SLA_NACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58
#define MT_SLA_ACK 0x18
#define MT_SLA_NACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NACK 0x30

class I2C {
  public:
  I2C();
  void begin();
  void writeRegister(int device, unsigned char address, unsigned char value);
  byte readRegister(int device, unsigned char address);
};

extern I2C i2c;

#endif
