#ifndef APIBNO_H
#define APIBNO_H

#include <termios.h>
#include <stdint.h>

// I2C Specifications ----
#define I2CPORT "/dev/i2c-1"
#define I2CADDR 0x4A
#define I2CXSPEED 100000
// ------------------------------

// SHTP cmd channel: byte-0=command, byte-1=parameter, byte-n=parameter
#define CHANNEL_COMMAND      0
// SHTP exec channel: write 1=reset, 2=on, 3=sleep; read 1=reset complete
#define CHANNEL_EXECUTABLE   1
// Sensor Hub control channel for sensor config commands and responses
#define CHANNEL_CONTROL      2
// Input Sensor reports only sends data from sensor to host
#define CHANNEL_REPORTS      3
// Wake Input Sensor reports sends data from wake sensors to host
#define CHANNEL_WAKE_REPORTS 4
// Gyro rotation vector in extra channel to allow prioritization
#define CHANNEL_GYRO         5

// Packets can be up to 32k.
#define MAX_PACKET_SIZE      32762 

#define MAX_METADATA_SIZE    9

// SHTP cmd channel: byte-0=command, byte-1=parameter, byte-n=parameter
#define CHANNEL_COMMAND      0
// SHTP exec channel: write 1=reset, 2=on, 3=sleep; read 1=reset complete
#define CHANNEL_EXECUTABLE   1
// Sensor Hub control channel for sensor config commands and responses
#define CHANNEL_CONTROL      2
// Input Sensor reports only sends data from sensor to host
#define CHANNEL_REPORTS      3
// Wake Input Sensor reports sends data from wake sensors to host
#define CHANNEL_WAKE_REPORTS 4
// Gyro rotation vector in extra channel to allow prioritization
#define CHANNEL_GYRO         5

// Control channel commands (BNO8X datasheet figure 1-30)
#define COMMAND_RESPONSE     0xF1
#define COMMAND_REQUEST      0xF2
#define FRS_READ_RESPONSE    0xF3  // Flash Record System read response
#define FRS_READ_REQUEST     0xF4  // Flash Record System read request
#define FRS_WRITE_RESPONSE   0xF5  // Flash Record System write response
#define FRS__WRITE_DATA      0xF6  // Flash Record System write data
#define FRS__WRITE_REQUEST   0xF7  // Flash Record System write request
#define PRODUCT_ID_RESPONSE  0xF8
#define PRODUCT_ID_REQUEST   0xF9
#define GET_TIME_REFERENCE   0xFB 
#define GET_FEATURE_RESPONSE 0xFC
#define SET_FEATURE_COMMAND  0xFD
#define GET_FEATURE_REQUEST  0xFE

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define REPORTID_ACC 0x01 // Accelerometer
#define REPORTID_GYR 0x02 // Gyroscope
#define REPORTID_MAG 0x03 // Magnetometer
#define REPORTID_LIN 0x04 // Linear Acceleration
#define REPORTID_ROT 0x05 // Rotation Vector
#define REPORTID_GRA 0x06 // Gravity
#define REPORTID_GAM 0x08 // Game Rotation Vector
#define REPORTID_GEO 0x09 // Geomagnetic Rotation
#define REPORTID_TAP 0x10 // Tap Detector
#define REPORTID_STP 0x11 // Step Counter
#define REPORTID_STA 0x13 // Stability Classifier
#define REPORTID_PER 0x1E // Personal Activity Classifier
#define REPORTID_IGR 0x2a // Integrated Gyro/Rotation

// Hardware selection method
#define USEI2CBCM 0x1
#define USEI2CDEV 0x2


struct Gyro_Open_t {
    char *dev;
    unsigned int addr;
    unsigned int xspeed;
    unsigned int fd; 
};


/* ------------------------------------------------------------ *
 * BNO080 measurement data structs. Data gets filled in based   *
 * on the sensor component type that was requested for reading. *
 * ------------------------------------------------------------ */
struct bno_acc_t {
   double adata_x;   // accelerometer data, X-axis
   double adata_y;   // accelerometer data, Y-axis
   double adata_z;   // accelerometer data, Z-axis
};
struct bno_eul_t {
   double eul_head;  // Euler heading data
   double eul_roll;  // Euler roll data
   double eul_pitc;  // Euler picth data
};
struct bno_qua_t {
   double quater_x;  // Quaternation data X
   double quater_y;  // Quaternation data Y
   double quater_z;  // Quaternation data Z
   double quater_w;  // Quaternation data W
};
struct bno_gra_t{
   double gravityx;  // Gravity Vector X
   double gravityy;  // Gravity Vector Y
   double gravityz;  // Gravity Vector Z
};

// --------------------------------
void  intHandler(int sig);

// Public Function pointers access point (associations in apiBno.c) 
//uint8_t (*bno_open)();
//void (*bno_close)();

void bno_init(uint8_t method);
uint8_t bno_reset();
uint8_t bno_errors();
uint8_t	bno_set_feature(uint8_t r_id,int period_usec);
uint8_t bno_read_event(struct State_t * state_p);

#endif 