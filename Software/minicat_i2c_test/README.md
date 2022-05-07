- Adding minicat_i2c_test.ino to test the I2C communication and Master (Arduino) - Slaves (Servo driver board and Gyroscope) present
- 
- After uploading the Sketch you can start the "Tools / Serial Monitor" (57600 baud)
- 
- the following result should be shown if all I2C devices are recognised:
Scanning...
I2C device found at address 0x40  ! (Servos driver board, I2C slave)
I2C device found at address 0x68  ! (Gyro board, I2C slave)
I2C device found at address 0x70  ! (Arduino board, I2C master)
done
