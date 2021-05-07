# gyroBNOx
A C linux interface to the Ceva BNO080 BNO083 FSM300 FSM305 motion IMU devices. 

# gyroWitMotion
A C based console program to setup and read data from a CEVA Hillcrest Bosch IMU Gyro devices. Specifically this has been tested with the BNO085 and FSM305 but I suspect will work with other similar Ceva/Hillcrest Motion devices. This program is targed to run on the Raspberry Pi and tested with a model 4. This program has been written to support I2C interface using the BCM2835 and also the /dev/i2c-1 device. There is a hardware abstraction layer in the apiBNO.c file so if you wanted add SPI or UART it should be pretty straight forwar - just generated specific open,send,read,close functions and align the function pointers in a section of code towards the top as:

  send_data = send_i2c_bcm;  
  read_data = read_i2c_bcm;  
  bno_open = open_i2c_bcm;  
  bno_close = close_i2c_bcm;  

Note at the time I am only requesting the report id 0x2A also known as “inputGyroRv” which is a fast rate combined rotation vector and gyro angular velocity. More could easily be added but this is not general purpose code. I might add more as I need them but I am on a time crunch. 

Also my project requirements are to post the data to a shared memory location /dev/shm/gyro0. You can comment out this part of the code if you like. 

# Synopsis

> ./bin/gyroBNOx


# Validation

I validated the gyro angular velocity reported via  report id 0x2A by fixing the BNO085 gyro to a turn table along with a Witmotion gyro. The results are shown here.   

![Gyro Traces](/docs/bno085_witmotion_gyro_traces.png)

Gyro1 trace is the BNO085 (actually FSM305) and Gyro2 trace is the Witmotion HWT905

The BNO device has slightly lower latency. It also supports a faster data cycle of up to 1000 Hz using the inputGyroRv or report id 0x2A. See plot below where the BNO085 picks up on a motion ramp a few samples before the Witmotion device.  

![Gyro Traces](/docs/bno_leading_impulse.png)


Note while evaluating the BNO Ceva Hillcrest with the Witmotion device I found a problem with the witmotion hwt905 device where if configured in gyro calibrate mode (with the compass) the Witmotion has erroneous jumps. You can see the notes here: 

https://github.com/troxel/gyroWitMotion  

I used bits and pieces of code I found various places on the web and there is some global data used in the api since the code I borrowed was being used on a MCU vice on a Raspberry PI. I may go back and clean this up when I get time.  
