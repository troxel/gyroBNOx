# gyroBNOx on Raspberry PI

A C linux interface to the Ceva BNO08x FSM300 FSM305 motion IMU devices to run on a Raspberry PI or similar processor. 

# gyroBno0x program. 

A C based console program to setup and read data from a CEVA Hillcrest Bosch IMU Gyro devices. iSpecifically this has been tested with the BNO085 and FSM305 but I suspect will work with other similar Ceva/Hillcrest Motion devices. This program is targed to run on the Raspberry Pi and tested with a model 4. This program has been written to support I2C interface using the BCM2835 library, the /dev/i2c-1 device, and the /devi2c-3 (gpio interface). There is a hardware abstraction layer in the apiBNO.c file so if you wanted add SPI or UART it should be pretty straight forward - just generated specific open,send,read,close functions and align the function pointers in a section of code towards the top as:

  send_data = send_i2c_bcm;  
  read_data = read_i2c_bcm;  
  bno_open = open_i2c_bcm;  
  bno_close = close_i2c_bcm;  

Also my project requirements are to post the data to a shared memory location /dev/shm/gyro0. You can comment out this part of the code if you like. 

# Getting started. 

You will need to download the BCM2835 library which can be found here

https://www.airspayce.com/mikem/bcm2835/

Install the bcm2835 by configure and make. If you don't want to use this library you can comment out the includes in the dot h files. This library works well with older Pi's and older OS. 

Next step is issue a 
```
  > make 
```
In the project directory. 

# I2C Issues. 

There has been significant issue with I2C compliance of the Bno chips and the I2C bus. In older PI's it is advisable to use the BCM2835 library. I have test with a Raspberry PI 4 and 64-bit bookworm PI OS and the BCM2835 library no longer works. But the I2C device does.  You can enabling the I2C bus with the following lines in the config.txt 

*dtparam=i2c_arm=on
dtparam=qi2c_arm_baudrate=400000*

On reboot this should build the /dev/i2c-1 device and then you should be able to go 

```
  >./bin/GyroBno.exe -d /dev/i2c-1
```
If that doesn't work (and it didn't in some cases) you can try the gpio device. Add the following line in the config.txt 

*dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2,bus=3*

This will create a device /dev/i2c-3 and you should be able to 

```
  >./bin/GyroBno.exe -d /dev/i2c-3
```
To get it work

# Synopsis
```
  > ./bin/GyroBno.exe [-d i2c-device] [-f feature-set-hex] [-p get-product-id]

  > -d : Must be either bcm2835 or existing i2c device in /dev
  > -f : Feature set hex number of ST-2 manual. It defaults to feature set 0x2A.
  > -p : retrieved product id (ie firmware version of Bno08x chip 
```

# Validation

I validated the gyro angular velocity reported via  report id 0x2A by fixing the BNO08x gyro to a turn table along with a Witmotion gyro. The results are shown here.   

![Gyro Traces](/docs/bno085_witmotion_gyro_traces.png)

Gyro1 trace is the BNO085 (actually FSM305) and Gyro2 trace is the Witmotion HWT905

The BNO device has slightly lower latency. It also supports a faster data cycle of up to 1000 Hz using the inputGyroRv or report id 0x2A. See plot below where the BNO085 picks up on a motion ramp a few samples before the Witmotion device.  

![Gyro Traces](/docs/bno_leading_impulse.png)


Note while evaluating the BNO Ceva Hillcrest with the Witmotion device I found a problem with the witmotion hwt905 device where if configured in gyro calibrate mode (with the compass) the Witmotion has erroneous jumps. You can see the notes here: 

https://github.com/troxel/gyroWitMotion  

I used bits and pieces of code I found various places on the web and there is some global data used in the api since the code I borrowed was being used on a MCU vice on a Raspberry PI. I may go back and clean this up when I get time.  

https://os.mbed.com/users/MultipleMonomials/code/BNO080//file/430f5302f9e1/BNO080.cpp/
