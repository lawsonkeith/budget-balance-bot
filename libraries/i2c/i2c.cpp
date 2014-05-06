/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:34 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/i2c/i2c.cpp,v $ 
 $Revision: 1.1 $
 $Log: i2c.cpp,v $
 Revision 1.1  2014/04/28 20:46:34  keithl
 Latest revision

*/
#include "i2c.h"
#include <Wire.h> 

I2C::I2C() {
}

void I2C::begin() {
  Wire.begin();
  /*PORTC |= ((1<<PORTC4) | (1<<PORTC5)); //set the i2c pins to internal pull up resistors
  TWBR = I2C_BAUD;                                  // We want 400kHz.
  //TWSR = TWI_TWPS;                                // Not used. Driver presumes prescaler to be 00.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
  (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
  (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
  (0<<TWWC);                                 //
  */
}    

void I2C::writeRegister(int device, unsigned char address, unsigned char value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}
byte I2C::readRegister(int device, unsigned char address) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(device,1);
  return Wire.read();
}


I2C i2c=I2C();

