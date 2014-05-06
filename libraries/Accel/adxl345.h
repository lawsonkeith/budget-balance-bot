/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:21 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/Accel/adxl345.h,v $ 
 $Revision: 1.1 $
 $Log: adxl345.h,v $
 Revision 1.1  2014/04/28 20:46:21  keithl
 Latest revision

*/
#ifndef adxl345_h
#define adxl345_h

#define ADXL_WHO_AM_I 0
#define ADXL_THRESH_ACT 0x24
#define ADXL_THRESH_INACT 0x25
#define ADXL_TIME_INACT 0x26
#define ADXL_ACT_INACT_CTL 0x27
#define ADXL_ACT_TAP_STATUS 0x2B
#define ADXL_BW_RATE 0x2C
#define ADXL_POWER_CTL 0x2D
#define ADXL_INT_ENABLE 0x2E
#define ADXL_INT_MAP 0x2F
#define ADXL_INT_SOURCE 0x30
#define ADXL_DATA_FORMAT 0x31
#define ADXL_DATAX0 0x32
#define ADXL_DATAX1 0x33
#define ADXL_DATAY0 0x34
#define ADXL_DATAY1 0x35
#define ADXL_DATAZ0 0x36
#define ADXL_DATAZ1 0x37

#define ADXL_CALIBRATION_SAMPLES 512 //number of samples to average to work out the offset

class ADXL {
public:
  ADXL(char device);
  void init();
  int getX();
  int getY();
  int getZ();
  bool dataReady();
  bool sleeping();
private:
  char _device;
  int _offsetX, _offsetY, _offsetZ;
  int readInt(char address);
  void calibrate();
};

#endif
