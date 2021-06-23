Here some hints about the Libraries you will need in order to compile the code for MiniCat :
- Libraries are code files needed to allow special hardware you add to your design to be controlled / managed by the Arduino Nano code
- To add Libraries you need to download their related code and #include them in your main .ino MiniCat Code
- To do so, open your Arduino and go to : Sketch/Include Library/Manage Libraries... then search for the Library you want to download, see details below 
- Libraries you then installed are stored per default under: Documents\Arduino\libraries\I2Cdev

Here the Libraries you need for MiniCat :

Official Arduino Libraries (already installed w/ the IDE) :

- "Wire" / #include <Wire.h> : 
This official Arduino library comes already with your IDE installation and allows you to communicate with I2C devices 
- the MiniCat ArduinoNano board 1 (Master) has the ServoDriver board (Slave) and the Intertial Measurement Unit IMU board (Slave) as I2C devices
  Note that for the IMU board it is not including directly the official I2C Wire lib but is is included through I2Cdev.h (for supporting other boards too), see below
- the MiniCat ArduinoNano board 2 (Master) has the OLED boards (Slave) as I2C devices

- "EEPROM", #include <EEPROM.h>, Library to access the external EEPROM Memory present on the Arduino Nano board. 
This EEPROM memory is non volatile (means robust to MCU Hard Reset) and allows to store the Servomotors calibration data (you need to calibrate only once !)

Additional Libraries specific to MiniCat needs :

- "IRremote 2.2.3" / "#include <IRremote.h>": 
Need to be installed to control MiniCat with an Infrared remote control through the Infrared receiver on Arduino Nano

- "Adafruit PWM Servo Driver Library 2.4.0" ,#include <Adafruit_PWMServoDriver.h> :  
Need to be installed for communicating over I2C to the Servo Driver Board 
(board needed to extend the Arduino Nano as it has only 6 x PWM outputs and we have 8 servo motors to control (2x per MiniCat leg))

- "MPU6050", #include #MPU6050_6Axis_MotionApps20.h" :
Need to be installed to operate the MPU (gyro+accelerometer) via I2C, this MPU Library was written by Jeff Rowberg as well as the custom I2C communication lib below
- "I2Cdev", #include "I2Cdev.h"

- "U8g2 2.28.10", #include <U8g2lib.h> : 
Need to be installed for communicating over I2C to the OLED displays 
Remark: both displays used here have the same I2C address : 0x3C, can be modified to 0x3D by unsoldering a resistor, see in your OLED display spec 


