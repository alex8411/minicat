MiniCat_2.0_PCB_1_0.fzz is the 2 layers PCB (Printed Circuit Board, top and bottom) for the MiniCat 2.0 robot.

A PCB basically spares you all the manual wires connections that were done in MiniCat 1.0.

This avoids manual connections errors and add robustness to the prototype.
In that way you can concentrate on the different processing, actuators and sensors.

It allows a highly modular / easy to assemble robot.

You can connect all the modules step by step and test them with the related Arduino Software :
- Module 1 : Arduino main Microcontroller board
- Module 2 : Buzzer
- Module 3 : Infrared Receiver
- Module 4 : I2C bus connection between main Arduino Microcontroller board and MPU (Gyroscope)
- Module 5 : MPU (Gyroscope)
- Module 6 : Servo Motors and Driver board (tested together with MiniCat final Software)

MiniCat_2.0_PCB_1x_LAYER_1_0.fzz implements the same connections but this time on 1 top layer only.
This was optimized in order to be able to prototype the PCB on a CNC Mill.

