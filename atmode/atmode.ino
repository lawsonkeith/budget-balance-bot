/*

AUTHOR: Hazim Bitar (techbitar)
DATE: Aug 29, 2013
LICENSE: Public domain (use at your own risk)
CONTACT: techbitar at gmail dot com (techbitar.com)#


K Lawson 22 Mar 2014
--------------------

Use this to program the HC-05 module on breadboard before it's inserted into the robot.

http://www.techbitar.com/modify-the-hc-05-bluetooth-module-defaults-using-at-commands.html

WIRING

•HC-05 GND --- Arduino GND Pin
•HC-05 VCC (5V) --- Arduino 5V
•HC-05 TX --- Arduino Pin 12 (soft RX)
•HC-05 RX --- Arduino Pin13 (soft TX)
•HC-05 Key (PIN 34) VCC (3.3V)

Power cycle the HC-05 to put it in command mode.  The LED flashed slower @ .5Hz  then:

Connect arduino serial monitor @ 9600 CR/LF
The arduino talks to the HC-5 at 38K4.  This is allways the case in cmd mode.

AT?
  says "OK"
AT+NAME=HC-05-BALANCE-BOT
  changes name of device
AT+UART=57600,0,0
  change Baud rate to be faster

Remove Key Pin and power cycle HC-05 to put back into data mode.

Note -  we need to use the software UART since talking to the HC-05  sharing the arduinos UART doesn't work very well.

*/

#include <SoftwareSerial.h>

SoftwareSerial BTSerial(12, 13); // RX | TX

void setup()
{
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
}

void loop()
{

  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available())
    Serial.write(BTSerial.read());

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
    BTSerial.write(Serial.read());
}


