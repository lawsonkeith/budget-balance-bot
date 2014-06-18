Project: Budget Bot lite
Author:	 K Lawson
Date:	 6 May 2014

Description
-----------
Low cost balance bot platform.  This is a small; easy to build sub �50
robotic platform that can be controled (driven) using a bluetooth 
smartphone app.  The platform is optimised for those on a budget who 
still demand a cool looking robot with fake alloy wheels!!

Uses arduino mini pro; 9 DOF IMU and HC-05 as well as a L9110 motor controller.

 ![](https://github.com/lawsonkeith/bblite/raw/master/wiki/Image1.JPG)



# Files
Check out the awesome performace!

 http://youtu.be/g5DNjcppkYU

Schematic and BOM:

 https://github.com/lawsonkeith/budget-balance-bot/blob/master/A0015_schematic.pdf (TinyCAD)
 https://github.com/lawsonkeith/budget-balance-bot/blob/master/BOM.txt

# Introduction
The aim of this project is to create a low cost balance bot platform that is easy to build and robust in construction.  Typically it should be possible to build for around £50 but it depends largely on the number of items in the BOM you already have or can scrounge.  

The community is invited to take this project as the departure point and come up with a better / cheaper budget balance bot!

# Robot platform overview
To keep costs low the robot uses cheap motors with no encoders (as they double the cost).  Cable trunking is used as the chassis (you can use a variety of similar alternatives) and the sub modules all fasten together via a single wiring harness and are secured using velcro or tie wraps.  There is no complex assembly and it is easy to swap modules out in the event of failure.  A shock absorber is mounted towards the top of the robot to prevent damage during falls.

8 AAA batteries are used to power the robot and it uses an Arduino mini pro controller.  The robot can be controlled using any android phone once an app has been downloaded from the google play store.


# Electronics
The following modules form the electronic control system; they are all easily obtainable in module form and just need soldering together with a few wires.

1. An Arduino pro mini 3.3V 16MHZ - Main control micro-controller.
2. 9 DOF IMU - Provides pitch of robot.
3. A L9110 H Bridge - Allows the Arduino to control the motor speed and direction.
4. An HC-05 bluetooth modem allows the robot to be controlled by the smartphone.

The Arduino micro controller has a variety of onboard peripherals that are used to perform the following tasks:

1. Control of the motor H bridge - using PWM
2. Decoding commands from the bluetooth modem - using a software UART
3. Reading accelerometer and rotational velocity from the IMU - using I2C
4. Calculating the vehicles pitch - using a filter algorithm
5. Controlling the vehicles movement (kinematics) - more algorithms

The balance bot is an inverted pendulum; for the robot to balance it first needs to know it's current chassis pitch.  The robot knows it's own balance point i.e the point at which it balances so the difference between this and the vehicles current pitch is the pitch error.

The IMU is used to provide a fast pitch estimate; this device gives us acceleration and angular velocity and we work out pitch from this. 

To be able to correct this error the Arduino drives the two motors forwards or backwards using the H bridge at a rate based on the error (The micro can't drive enough current to do this and also it doesn't like driving inductive loads).    

The Modem receives commands from a smartphone that can then drive the robot as commanded by the operator.


# Drive system
A L9110 H bridge controls the motor speed an direction independanty.  For a balancing robot the motors used are probably the most important thing to get right.

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/hbridge.png)

We control the motors using a dual H bridge IC which enables is to proportionally control the motors speed and direction.  We need this as the micro itself doesn't have enough power to drive a motor and there are some issues with inductive voltages as well that would eventually destroy the micro.   An H bridge IC is much cheaper than making our own circuit and gives us 4 usefull states (fwd / back / 2x brake) rather than the 16 states of a DIY h-brige; of which some are short circuits.  The L9110 also has protection agains inductive spikes.  It's fairly trivial to drive; you will see I've used one PWM for rate and one GPIO for direction as I experienced some issues with the arduino wire library not driving quite right when using 2 PWMs to control a single motor.  When you do this it's necessary to invert the duty cycle in one direction due to the way the logic works.


The angular acceleration of the robot when it falls is inversely proportional to how high the centre of mass is.  You can see this effect when you try and balance a short or a long object with your hand; the longer object will be easier to balance.  The motor torque must therefore be sufficient to overcome this; the motor speed must also be high enough to allow stabilisation to occur before the motor goes full speed.  There must also be enough speed in reserve for the motor to drive the robot around.

This robot uses 6V DC motors 133RPM gearmotors, these motors are available in a variety of ratios but this one seems to work best.  DC motors are good because when balancing they don't consume much current (unlike stepper motors which allways consume max current); you can also over-drive the motors since most of the time they aren't driven 100% all the time so won't burn out.  I've used large motors which have the disadvantage of being heavy which lowers the robot centre of mass but the more important advantage of linearity particularly at breakout.

Breakout is the point at which the wheels start turning; although the torque/voltage relationship is linear the speed/voltage relationship is not because the torque has to overcome internal gearbox friction and stiction etc as well as wheel inertia in order for the wheels to rotate.  I've used a formula to get rid of this non-linearity in the softare; basically you add an offset and scale the demand from the PID controller.  The PID controller would struggle to deal with this non linearity otherwise and the robot wouldn't balance very well.

When encoders aren't being used the breakout becomes an issue as 'sticky' gearboxes mean the wheels don't start turning predictably and the robot turns (unintentionally) a lot as one wheel turns more than the other.  A lot of gearmotors were tred before I got to these ones which balance nicely.  

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/torque.PNG)

Note - For any particular balancing robot platform you need to ensure the motors have enough torque for the robot center of mass and intertia charictaristics as well as having enough speed to handle how fast you want to drive it.  For any gearmotor type there will be an optimum gear ratio to use that combines max speed and torque optimally for your design; for example 600rpm motors would have insufficient torque to balance the robot unless the center of mass was raised significantly. 

To do a really good job you need to use encoders and a powerful motor which unfortunately can't be done within the £50 price constraint.  


# Theory of operation

In order to stay upright the robot controls it's pitch error as discussed previously, this is derived as follows:

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/ange%20calc.PNG)

1. The Y axis of the accelerometer will read 1g when vertical.  Using small angle approximation you can work out fairly easily that the the X axis of the accelerator is proportional to pitch in radians for small angles (which is all we care about on a balancing platform since it'll fall over past 20 degrees anyway).
2. The integral of the rotational velocity from the gyro also gives us pitch.

On their own these sensor values are fairly useless because:

1. The accelerometer x axis also reads forwards and backwards acceleration.
2. The gyro also integrates errors so drifts over time.  The gyro also has no sense of 'up' i.e it will take the point it's turned on as it's 0 reference.

To solve this problem we use a sensor fusion algorithm to combine the good points of both sensors to come up with an estimate of pitch that can be used for stabilizing the platform.  There are many ways of doing this but I've used a complementary filter which basically uses the gyro for short term and the accelerometer for long term pitch estimation.  This is easy to understand and doesn't waste much CPU power; Kalman filters and the direction cosine matrix are alternative methods of achieving better results but are more complex, harder to understand and need more CPU power.  Since we are using and 8 bit micro we can't do anything too complex here.

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/fiter.PNG)

The micro controller uses a PID control algorithm that corrects the pitch error by looking at the error (proportional); sum of the error over time (integral); rate of change of the error (derivative) and driving the motors so that they reduce these 3 error components.  Each of these terms has a constant associated with it that controls how much the controller acts to reduce that part of the error.  It is necessary to come up with optimum settings for these constants; this process is called 'tuning'.  These consants are hard coded into this software fork but if there are any changes to the robot platform it will need to be re-tuned; this is a process whereby you alter the PID constants to try and obtain those optimum for stability.    

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/pid.PNG)

A second PID controller is then used to drive the motor forwards or backwards depending on the command; this is done by controlling the vehicles pitch reference point.  A second controller gives better results than just varying the platform pitch as it can take the vehicles dynamics into account.  If we just controlled the pitch directly the robot would tend to keep accelerating till it tipped over.


# Assembly

Assembly is split into 4 main stages, print the main robot schematic and BOM then follow the following steps.

## 1.Motor assembly

Tools- 3mm Terminal driver; Pozi screwdriver; Hacksaw, Soldering stuff, Electrical tape


Items - M1,2; O10; C1,2; O5

a. Solder the capacitors to the motors and also solder 15cm worth of cable to the motors as well.  We will trim this to the exact length later on so just make sure they aren't too short.  Add electrical tape to prevent any possible shorts.

b. Trim (using hacksaw) the card tube so that it can mount the motors.  Use closed cell foam as a spacer inside the motor mount to prevent the terminals from shorting.  A small opening is required to allow the cables to be connected to the motor controller.

c. Test each motor with a 9v battery to make sure it works.  The motors should be loosely held in place; once mounted with tie wraps they will be held firmly in place. 

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/DSC_0175.jpg)

## 2.Main Chassis

Tools- Files, Hacksaw, Drill

Items- O8, 09, 07, 03, 05, S1

a. Cut trunking to 210mm length.

b. Using file and hacksaw create a curved mount for the motors; it doesn't need to be prefect but it must hold the motors in position once they are tie wrapped into place.

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/DSC_0179.jpg)

c. Use a file to create a mounting mole for the switch S1

d. Fashion item 08 into a mount for the IMU and affix to the robot.

e. Mount the IMU

f. affix velcro to mount sub modules.

g. The battery holder will require some padding to stop it moving around.

h. Cut a 15mm and 95mm cover.

i. Use closed cell foam and insulating tape to make a bumper.  This needs to fit around the battery holder.

## 3.Wiring harness and final assembly

Tools- Soldering stuff, Wire cutters, superglue, rubber sleeving or heat-shrink.

Items- B1, F1, S1, O1, O2

  

a. Using the wiring schematic as a reference wire the 6w header sockets to the 24 way DIP socket switch fuse and battery (This is the most difficult bit allow at least an hour!)  I used a small piece of vero-board as a hub for all the power wires which helps with the routing. 

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/DSC_0182.jpg)

b. Check your work with a meter.

c. The lengths are important here.

e. Fit Harness to chassis and secure the 24w DIP with a tie wrap.

f. Affix remaining electronic modules.

f. Tie wrap the motors and connect to the H bridge.  Tip - Mark them up once you know which they are connected correctly.

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/DSC_0183.jpg)

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/DSC_0184.jpg)



## 4.Using the software

a. Search for "E Robot Remote" on google play.  The balance bot is compatible with it.

b. Configure the HC-05 module using the pre-supplied arduino sketch 'at mode' follow the commands in there to configure the HC-05 module to the correct baud rate.

c. Open the main balance bot sketch and program the arduino.

d. Once powered up; lie the robot flat electronics face up - the motors should spin for a second in the same direction and the platform moves in the direction of the batteries.

e. raise robot; it should balance.

f. If not you may need to re-calibrate the motors using the supplied triangle generator.



# Android control

This robot is compatible with the 'E-robot remote' (By WAHIBI of www.e-robot.com) app that can be (re)used to control this robot.  Run the app then connect to the balance bot robot.  The joystick controls movement and the yellow and red buttons can be used to adjust pitch 0 degree reference point.

![](https://github.com/lawsonkeith/bblite/raw/master/wiki/erobot.png)

Or just write your own!

# Future work

Over to you community...

1. Make the dynamic control more stable; it's fairly easy to make it fall over with the android app at the minute.
2. Make it go into outer space like in the picture at the start of the wiki.
3. Use Kalman Filter or DCM for pitch estimate; a move towards fixed point math may be required to accomplish this.
4. You are encouraged to come up with your own take on the chassis - Corrugated cardboard / plywood / Perspex / laser cut chassis / 3D printed chassis / shampoo bottle etc.
5. Adding some obstacle avoidance sensors.  
6. Adding inertial navigation so the android remote tells it to advance in say 10cm increments.
7. Doing something with the magnetometer.
8. Motor encoders would improve the dynamic stability; I omitted them mainly due to cost and complexity.  
9. Develop your own android controller. 
10. Find some better motors.
11. Battery voltage compensation - currently when the battery drops the PID gets de-tuned.
12. Replacing the bluetooth with an IR sensor would free up cash for better motors?
13. Faking how an encoder would be used in a compound PID with one or more IMUs.
14. Center of mass - to be honest is far too low; a taller chassis will get round this at the minute its about 5cm above the wheel axis.  The only reason for it to be the low was to make it look cool.


-Elongating the chassis will deliver more stability as will adding weight towards the top of the vehicle.  The idea with the chassis was to make it out of anything you have lying around.  I just happened to have access to lots of cable trunking; it is a very good material though.
