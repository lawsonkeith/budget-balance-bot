/*
 ===================================================================
 CSCI:    BalanceBot 
 ===================================================================

 Description
 -----------
 Simple balance bot project using Arduino 3.3V Atmel328P 
 Gyro, Accelerometer and L9110 motor driver.  2x 133 (@ 6V) rpm
 DC motors control.  Powered by 8AA batteries @ 10-11V.
 
 Create an IMU to control  pitch of vehicle with motor 
 controller.
 
 Turn on procedure
 -----------------
 Turn on.
 Lie unit flat.
 When motor kicks bring verical.
 
 NOTE - Otherwise accelerometer calibration isn't great.
 
 Calibration
 -----------
 0. Calib breakout; using the triangle generator make sure the crossover is smooth; the motors just about should stop at 0
 The offset is generally the figure to watch.
 1. Use triange wave to compensage for drift.  Bot should hold position and not rotate.
 2. Hang bot with no prg cable; adjust mtg angle till wheels oscillate under PID control (just P)
 3. Tune PID
 a. P till it can stand up for 2-5s
 b. I till it can resist disturbance
 c. D to get rid of over reactions
 4. Repeat as nexessary from 1 when it b
  
 ===================================================================
 */
//#define ITG3200_ADDRESS_ALTERNATE

#include <Wire.h> /

#include "i2c.h"
#include "itg3200.h"
#include "adxl345.h"
#include <SoftwareSerial.h>

#define TEST_CONTROL 0  // 1= triangle test pattern     0 = IMU slabilisation
#define PRINT_CHART 1   // 1= outpput ser diagnostics   0 = no diagnostics
// 6 xAA #define MTG_OFFSET 1.9  //+ lean forwards              - leans back
#define MTG_OFFSET 4.8  //+ lean forwards              - leans back


#define GYRO 0x68        // I2C addresses
#define ACCEL 0x53

// Phone command button defines
#define NO_CMD 0

#define CNR_TL 1
#define CNR_TR 2
#define CNR_BL 3

#define PAD_TL 4
#define PAD_TR 5
#define PAD_BL 6
#define PAD_BR 7

#define PAD_YL 8
#define PAD_RD 9
#define PAD_BU 10
#define PAD_GN 11



ADXL adxl(ACCEL);
ITG itg(GYRO);

SoftwareSerial BTSerial(12, 13); // RX | TX

int count=0;


/*
 ===================================================================
 CSU:    setup()
 ===================================================================
 Desc:   Setup the arduino IO pins
 
 Arduino 328P 16MHz 3.3V
 -----------------------
 a4,a5    IMU  ITG-3200GYRO / adxl345 accelerometer I2C link  
 
 Motor controller: L9110
 a3     Stbd motor    a-1a   mag           
 a9                   a-1b  (dir)
 a10    Port motor    b-1b  (dir)      
 a11                  b-1a   mag
 
 HC-05:
 12 Tx 
 13 Rx     Soft serial port for bluetooth
 
 LED
 13 Gn
 ===================================================================
 */
void setup() 
{
  //required to talk to HC-05 on soft UART
  BTSerial.begin(57600);  // HC-05 default speed in AT command more

  // motor dir
  pinMode(9, OUTPUT);   //motor dir
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT);   //motor dir
  digitalWrite(10, LOW);
  
  // motor mag
  analogWrite(3, 0); 
  analogWrite(11, 0);//p
  
  // i2c
  i2c.begin();
  adxl.init(); //acc
  itg.init();  //gyro
  itg.wake();
 
  // terminal
  Serial.begin(57600);
  Serial.println("I2C initialised.");

  //led
  pinMode(13, OUTPUT); 

  // accel
  while (!itg.dataReady()) {
  }

  unsigned char whoami=i2c.readRegister(ACCEL,ADXL_WHO_AM_I);
  Serial.print("Accel: ");
  Serial.println(whoami,HEX);

  whoami=i2c.readRegister(GYRO,ITG_WHO_AM_I);
  Serial.print("Gyro: ");
  Serial.println(whoami,HEX);
  Serial.print("Temp: ");
  Serial.println(itg.getTemp());
  delay(2000);

}//END Setup



/*
 ===================================================================
 CSU:    loop()
 ===================================================================
 Desc:   main IO loop for baance bot, read the accelerometer and
 gyro; formulate a pitch solution  using a complementary filter
 then feed this into a PID alorithm and eventually control the
 motors with PWM to try and set robot pitch to 0.
 
 A debug loop can print parameters to 'SerialChart' program
 or 'Serial Monitor' for tuning purposes. 
 
 ===================================================================
 */
void loop() 
{
  static unsigned long deltaTime, currentTime, previousTime;
  static byte  loop2;
  static float PitchGyro, RotVel, Lean, Velocity,Distance,DSetpoint,VelError,VelCtrl;
  static int MotorDrive;
  static int Count;
  static int Xval, Yval;
  static byte Command;
  static float MtgOffset = MTG_OFFSET;



  // ================================================================
  // Max 4-500 Hz loop
  // Constrained by Accl / Gyro sampling
  // ================================================================
  if (1) 
  {
    // ==== Freq Count =========
    Count++;
    if((Count > 490) || (Command != 0) || (Yval != 0))
      digitalWrite(13, HIGH);//on
    else
      digitalWrite(13, LOW);//off

    if(Count > 500)//cycle time 
      Count = 0;

    EstimatePitch(&PitchGyro, &RotVel);

    currentTime = micros();
    deltaTime = currentTime - previousTime;

  }//END MAX Hz Loop



  // ================================================================
  // 100 Hz task loop
  // Control wheels with PID
  // ================================================================
  if (deltaTime > 10000) 
  { 
    //+f/-a [512]      +s/-p [512]       Command returns btn pressed
    ReadPhoneCommands(&Xval,&Yval,&Command);

    //dynamic setpoint adjustment using app buttons move bias
    if(Command == PAD_YL)
      MtgOffset += .1;
    if(Command == PAD_RD)
      MtgOffset -= .1;


    if(! TEST_CONTROL)
    {
      // @@@@ Control pitch off IMU @@@@@@@@

      // Estimate platform velocity and distance 
      // use 1st order filter to gain velocity
      Velocity = Velocity * 0.9 +  (MotorDrive * .0001); // +/-1    (-1 = going fully fwd)  
      

      // Note - Increasing pitch (error) makes vhicle lean forwards
      // PID control on  pitch; asssume 0 is setpoint .: pitch is error +/- 1000 drive
      MotorDrive = getPID( (PitchGyro + MtgOffset) + VelCtrl ,20,0.6,75);

      // As motor moves moves fwds overdrive the motors to stop runaway and let the gearbox slow it down
      //MotorDrive += Velocity * 100;

      DriveMotors(MotorDrive , Xval * 0.1 /*rot*/ , PitchGyro); // +/-1000 (-1000 =  ful fwd)




      // change pitch to control velocity

      // compare phone demand vel to actual vel to get a velocity error
      // we need this to control speed else it keeps over accelerating and
      // falling over.
      // Then use a 2nd pid to control the pitch setpoint of the platform.  The D is the initial 'kick' to get it moving.
      VelError = Velocity - (Yval * -.001) /* +/-1   +/-.5 to give error +/- 0.5*/;
      VelCtrl = getPID2(VelError,4,.002,30);
      
    }//END Balance ctrl
    else
    {
      delay(100);
      // @@@@ TRIANGLE Generator @@@@@@@@
      // used for validating motor scaling; the bot shouldn't drift too much and the motors should stop at low demands.
      // the motor breakouts should also be repeatable.
      //
      static int Sign = 1;

      MotorDrive+=Sign;
      if(MotorDrive > 100)
        Sign = -1;
      if(MotorDrive < -100)
        Sign = 1;

      DriveMotors(MotorDrive,0,0); // +/-1000      
    }//END triangle wave ctrl

    previousTime = currentTime;
    loop2++;
  }//END .1KHz Loop



  // ================================================================
  // 10Hz slow task loop
  // Misc stuff
  // ================================================================
  if (loop2 > 10) 
  {    
    loop2 = 0;
    if(PRINT_CHART)
    {
      //Serial.print("P");
      //Serial.print(PitchGyro + MTG_OFFSET);
      Serial.print(" , RV");
      Serial.print(RotVel);  
      Serial.print(" , V");
      Serial.print(Velocity);  
      Serial.print(" , DST");
      Serial.print(Distance);  
      Serial.print(" , MD");
      Serial.println(MotorDrive);  
      Serial.print(" , BTX");
      Serial.print(Xval); 
      Serial.print(" , BTY");
      Serial.println(Yval);    
    }
  }//END 10HZ loop
}//END Loop




/*
 ===================================================================
 CSU:    EstimatePitch()
 ===================================================================
 Desc:   Estimate pitch from the IMU.  This is done with a complementary
 filter using the acceleraometer and the gyro.  This is then
 put through a 1st order filter.
 Rotational velocity is also passed again after it's gone thorugh a 
 1st order filter.
 
 return  Pitch (Deg) and rotational velocity (Deg/s)
 
 ===================================================================
 */
void EstimatePitch(float *Pitch, float *PitchDerivative)
{
  static float PitchGyro, PitchAcc, FilterPitch, RotVel;
  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;

  // @@@@ read gyro; accel etc up to 1KHz @@@@

  // Accel
  // 4mg / LSB
  while (!adxl.dataReady()) {     
  }
  ax=adxl.getX();

  // Gyro (degs/sec)
  while (!itg.dataReady()) {     
  }
  gy=(float)itg.getY();

  // mount:_fw_
  //     p/ '  \s
  //      ==af==
  //      ||||||
  //
  // mount IMU so pins point down and green LED is fwd


  // @@@ ITG-3200GYRO @@@
  // lower level: already in rads/s
  // from the sensor :  14.375 MSBs = 1deg/s
  // from the driver  it's 1 deg/sec
  //
  PitchGyro += gy/*Deg/sec*/ * -0.01 /* 1/f */;
  RotVel = RotVel * 0.9 + gy * -0.1; //avg rotational velocity to pass to PID
  // @@@ adxl345 accelerometer @@@
  // 4mg per LSB
  // pitch is proportional to the x axis for small angles < 30 Deg. i.e sin(x) = x
  // 
  PitchAcc = ax /*sin(x)==x where angle << 30 in rads*/ * 0.004 /*mg to g*/ * 57.2958 /*rad->Deg*/ ; 

  // @@@ Complementary filter to get final pitch solution @@@
  //
  PitchGyro =  0.96 * PitchGyro + 0.04 * PitchAcc /*mtg offset*/; 

  // Limit the value of the pitch to prevent errors knocking it out of kilter
  //
  if(PitchGyro > 80)
    PitchGyro = 80;
  if(PitchGyro < -80)
    PitchGyro = -80;

  *Pitch = PitchGyro;
  *PitchDerivative = RotVel;
  
}//END Estimate Pitch



/*
 ===================================================================
 CSU:    getPID()
 ===================================================================
 Desc:   A PID implementation; control an error with 3 constants and
 return a rate +/- 1000.
 
 ===================================================================
 */
int getPID(float error,float kP,float kI,float kD)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr; 


  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -900, 900);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  Output = kP * error + kI * errSum + kD * dErr;       
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitFloat(&Output, -1000, 1000);

  return( Output );

}//END getPID



/*
 ===================================================================
 CSU:    getPID2() - PITCH CTRL
 ===================================================================
 Desc:   A PID implementation; control an error with 3 constants and
 return a rate +/- 10.
 
 ===================================================================
 */
int getPID2(float error,float kP,float kI,float kD)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr; 

  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -10, 10);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  Output = kP * error + kI * errSum + kD * dErr;       
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitFloat(&Output, -10, 10);

  return( Output );

}//END getPID

/*
 ===================================================================
 CSU:    DriveMotors()
 ===================================================================
 Desc:   Drive port / stbd motors fwd or backwards using PWM.
 A breakout calc is needed to linearise the response since
 torque is proportional to voltage on a DC mottor the wheels
 don't move on the lower 25% range.
 
 A L9110 IC controls the motors and 2 PWMs are used.  2 DOs control 
 direction.
 
 IA(DO) IB(PWM) Motor State 
 L      L       Off 
 H      L       Forward 
 L      H       Reverse 
 H      H       Off 

 Inputs
 -----
 DriveVal    +/-1000   - = fwd
 Rotation    +/-100
 Pitch       Pitch of vehicle in degrees; once fallen over stop demands
 ===================================================================
 */
void DriveMotors(int DriveVal,int Rot, float Pitch)
{
  int Mag;
  int PDrive, SDrive;

  PDrive = DriveVal - Rot;
  SDrive = DriveVal + Rot;

  LimitInt(&PDrive,-1000,1000);
  LimitInt(&SDrive,-1000,1000);
  
  //Crash detection
  if((Pitch > 45) || (Pitch < -45))
  {
    //kill motors; await rescue
    analogWrite(3, 0); 
    digitalWrite(9, LOW);
 
    analogWrite(11, 0);//p
    digitalWrite(10, LOW);

    return;
  }

  //this routine accemps a normalised input of +/- 1000.  This is then scaled to a PWM drive
  //of 0-255.

  //calc breakout using excel.  Here 0-1000 is n-255 to overcome breakout
  //this means the motor can transition from forwards to backwards smoothly
  //n.b the motors should stop in at 0.
  //

  //  a3     Stbd motor    a-1a           
  //  a9                   a-1b(dir)
  Mag = 0.223 * abs(SDrive) + 32 ; // 8 AAA
  analogWrite(3, Mag);//s

  if(SDrive > 0)
  {
    // fwds
    digitalWrite(9, HIGH);
    analogWrite(3, 255 - Mag);
  }
  else
  {
    // backwards
    digitalWrite(9, LOW);
    analogWrite(3, Mag);
  }  

  // a10    Port motor    b-1b(dir)      
  // a11                  b-1a
  Mag = 0.223 * abs(PDrive) + 32 ; // 8 AAA

  if(PDrive < 0)
  {
    // fwds
    digitalWrite(10, HIGH);
    analogWrite(11, 255 - Mag); //you haave to do this; look at the truth table
  }
  else
  {
    // backwards
    digitalWrite(10, LOW);
    analogWrite(11, Mag);
  }  
}//END DrivePortMotor









/*
 ===================================================================
 CSU:    ReadPhoneCommands()
 ===================================================================
 Decodes the command string from the E-ROBOT android controller app.
 
 The app sends out a standard binary packet of about 30 bytes:
 
 1. Once when any of the on screen buttons are pressed.
 2. Once when any of the buttons are released.
 3. The joystick is held; (this is a stream of data)
 
 The frame has a header; the analog data is contained in 2 bytes; the rest of the data is in a single byte as a 1 or 0.
 
 Note  - The HC-05 has to be confugured previously to talk at 57600 baud (default is 9K6 or 38K4).
       - The HC-05 must be in slave mode with they Key key not connected.
       - The HC-05 can be confugured with another sketch; see this folder for it.
       
 ===================================================================
 */
void ReadPhoneCommands(int *Xval, int *Yval, byte *Command)
{  
  static int a,b,c,d;    // char buffer
  static int framecnt;   // framecounter
  static unsigned long timer;
  int val;              

  *Command = NO_CMD; 

  // If got char from bluetooth
  while (BTSerial.available())
  {  
    timer = millis();

    //barrel shift
    d=c; 
    c=b; 
    b=a;    
    a=BTSerial.read();

    if(framecnt > 0)
      framecnt++;
    //else looking for header

    //locate header ID
    if((c == 6) && (b == 133) && (a == 26))
    {
      //start counting up new frame
      framecnt=1;
    }

    // count from header the target data field.  There is an analogue XY joystick and a variety of buttons
    // the buttons are in 3 groups - a pad with grey buttons - a pad with coloured buttons and 3 buttons in the corners
    //
    if(framecnt==3)
    {  
      val = (b + (a << 8)); 
      val -= 510; //+ 510 stbd    -510 port
      if((val < 1030) && (val > -1030) )      //catch packet collisions
      {
        if (val != 0)
          *Xval = val;// * .01 - 0.02; //+s/-p [512]  
      }
      else
        framecnt = 0;      // reset, data bad

    }
    else if(framecnt==5)
    {  
      val = (b + (a << 8)) ;
      val -= 510; //+ 510 Fore   -510 Aft
      if((val < 1030) && (val > -1030))  //catch packet collisions
      {
        if(val != 0)
          *Yval = val;// * .01 - 0.02; //+f/-a [512]
      }
      else
        framecnt = 0;            // reset, data bad

    }
    else if(framecnt==6) 
    {    
      if(a==0)
        *Command = CNR_BL;
    }
    else if(framecnt==10) 
    {
      if(a==0)
        *Command = PAD_TR;
    }
    else if(framecnt==8) 
    {
      if(a==0)
        *Command = PAD_TL;
    }
    else if(framecnt==18) 
    {
      if(a==0)
        *Command = CNR_TR;
    }
    else if(framecnt==14) 
    {
      if(a==0)
        *Command = PAD_BR;
    }
    else if(framecnt==12) 
    {
      if(a==0)
        *Command = PAD_BL; 
    }
    else if(framecnt==16) 
    { 
      if(a==0)
        *Command = CNR_TL;
    }
    else if(framecnt==20) 
    {
      if(a==0)
        *Command = PAD_YL;
    }
    else if(framecnt==24) 
    {      
      if(a==0)
        *Command = PAD_RD; 
    }
    else if(framecnt==22) 
    {
      if(a==0)
        *Command = PAD_GN;    
    }
    else if(framecnt==26) 
    {
      if(a==0)
        *Command = PAD_BU;  

      framecnt = 0; //reset,look for header  
    }    
  }

  if(millis() >  (timer + 500))
  {
    *Xval = 0;
    *Yval = 0;   
    framecnt = 0;
  }

}//END ReadPhoneCommands

/*
 ===================================================================
 CSU:    LimitInt()
 ===================================================================
 Clamp an int between a min and max.  
        
 ===================================================================
 */
void LimitInt(int *x,int Min, int Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt


/*
 ===================================================================
 CSU:    LimitFloat()
 ===================================================================
 Clamp a float between a min and max.  Note doubles are the same 
 as floats on this platform.
        
 ===================================================================
 */
void LimitFloat(float *x,float Min, float Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt

