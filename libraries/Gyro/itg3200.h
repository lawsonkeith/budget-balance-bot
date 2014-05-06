/*
 $Author: keithl $
 $Date: 2014/04/28 20:46:50 $
 $Source: C:/Documents and Settings/keithl.SMD/Desktop/A0015/bblite/bblite/libraries/Gyro/itg3200.h,v $ 
 $Revision: 1.1 $
 $Log: itg3200.h,v $
 Revision 1.1  2014/04/28 20:46:50  keithl
 Latest revision

*/
#ifndef itg3200_h
#define itg3200_h

#define ITG_WHO_AM_I 0
#define ITG_SMPLRT_DIV 0x15
#define ITG_DLPF_FS 0x16
#define ITG_INT_CFG 0x17
#define ITG_INT_STATUS 0x1A
#define ITG_GYRO_TEMP_OUT_H 0x1B
#define ITG_GYRO_TEMP_OUT_L 0X1C
#define ITG_GYRO_XOUT_H 0x1D
#define ITG_GYRO_XOUT_L 0x1E
#define ITG_GYRO_YOUT_H 0x1F
#define ITG_GYRO_YOUT_L 0x20
#define ITG_GYRO_ZOUT_H 0x21
#define ITG_GYRO_ZOUT_L 0x22
#define ITG_PWR_MGM 0x3E


class ITG {
public:
  ITG(char address);
  void init();
  float getX();
  float getY();
  float getZ();
  float getTemp();
  void sleep();
  void wake();
  bool dataReady();
private:
  char _device;
  int _offsetX, _offsetY, _offsetZ;
  int readInt(char address);
  void calibrate();
};

#endif
