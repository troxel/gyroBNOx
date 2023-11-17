#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h> // ftruncate
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <bcm2835.h>
#include <linux/i2c-dev.h>

#include "mstate.h"
#include "apibno.h"
#include "gyrobno.h"

#define I2CDELAY 200

extern uint8_t verbose;

// -- coversion factors ------
const double qp8 = pow(2, -8);
const double qp9 = pow(2, -9);
const double qp10 = pow(2, -10);
const double qp12 = pow(2, -12);
const double qp14 = pow(2, -14);
const double rad2deg = 57.2957795131;


// Global Data ----------------------------------------
// Todo reduce the use of globals particularlly these:
uint8_t shtpData[512]; 
uint8_t shtpHeader[4]; 
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;	//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel	uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3
uint8_t sequence[6]; // 6 SHTP channels. Each channel has its own seqnum

// > 10 words in a metadata, but we'll stop at Q point 3
unsigned int metaData[MAX_METADATA_SIZE];

extern uint8_t verbose; 

// Open details for i2c
struct Gyro_Open_t i2c_port;
      
struct timespec t1={0,0}, t2={0,0};
long diff_ns(struct timespec *, struct timespec *);

long rcnt=0;

// private functions. ------

// i2c bcm library functions 
uint8_t open_i2c_bcm();
uint8_t send_i2c_bcm(uint8_t *data,uint16_t data_len);
uint16_t read_i2c_bcm(uint8_t *data,uint16_t data_len);
void close_i2c_bcm();

// i2c-dev functions 
uint8_t open_i2c_dev();
uint8_t send_i2c_dev(uint8_t *data,uint16_t data_len);
uint16_t read_i2c_dev(uint8_t *data,uint16_t data_len);
void close_i2c_dev();

// Internal Packet functions
uint8_t bno_sendPacket(uint8_t channel, uint16_t datalen);
uint16_t bno_readPacket(void);

// Internal HAL read/send Functions 
uint8_t (*send_data)(uint8_t *,uint16_t);
uint16_t (*read_data)(uint8_t *,uint16_t);

extern uint8_t (*bno_open)();
extern void (*bno_close)();

uint8_t bno_get_prod_id();

// Util functions...
uint8_t quat2euler(double * qt, double * angles);

// -----------------------------------------------------
// Hardware abstraction
void bno_init(char *dev_i2c) {

   if ( strstr(dev_i2c,"bcm2835") != NULL ) {
      // User broadcomm 2835 library for i2c comms    
      send_data = send_i2c_bcm;
      read_data = read_i2c_bcm;
      bno_open = open_i2c_bcm;
      bno_close = close_i2c_bcm; 
      printf("Using Broadcomm2835 Library for i2c comms\n");   
   }
   else if ( strstr(dev_i2c,"i2c") != NULL ) {
      // ic2 but check for existence of file 
      if (access(dev_i2c, F_OK) == -1) {
         perror("Error! I2C Device does not exist");
         exit(EXIT_FAILURE);
      }

      send_data = send_i2c_dev;
      read_data = read_i2c_dev;
      bno_open = open_i2c_dev;
      bno_close = close_i2c_dev; 
      i2c_port.dev = dev_i2c; // For use in open_i2c_dev()
      printf("Using \'%s\' for i2c comms\n\n",dev_i2c);   
   }
   // Add SPI and UART if needed.  
}   


// -----------------------------------------------------
uint8_t open_i2c_dev() {

   // i2c_port.dev is set in bno_init() 
    i2c_port.addr = I2CADDR;
   i2c_port.xspeed = I2CXSPEED;
 
   if(( i2c_port.fd = open(i2c_port.dev, O_RDWR )) < 0) {
      printf("Error failed to open I2C bus [%s] %s\n", i2c_port.dev, strerror(errno) );
      exit(1);
   }
    if(verbose >= 1) printf("Debug: I2C bus device: %d [%s]\n", i2c_port.fd, i2c_port.dev);
   
    if (ioctl(i2c_port.fd, I2C_SLAVE, i2c_port.addr) < 0) {
      printf("Error! with I2C address [0x%02X] %s\n", i2c_port.addr, strerror(errno) );
      exit(1);
   }

   usleep(I2CDELAY);

   return i2c_port.fd; 
}

// -----------------------------------------------------
uint8_t open_i2c_bcm() {

   // i2c control using the bcm2835 library (also works with bcm2711)
   // Must compile with -l bcm2835

   i2c_port.dev = I2CPORT;   // set of now as it signals i2c usage... 
   i2c_port.addr = I2CADDR;
   i2c_port.xspeed = I2CXSPEED;
 
   //Initialize the library
   if (!bcm2835_init())
   {
      printf("bcm2835_init failed. Are you running as root??\n");
      return -1;
   }

   //Start the SPI0 controller
   if (!bcm2835_i2c_begin())
   {
      printf("bcm2835_i2c_begin failed. Are you running as root??\n");
      return -1;
   }
   
   bcm2835_i2c_set_baudrate(400000);

   bcm2835_i2c_setSlaveAddress(i2c_port.addr);

   if(verbose >= 1) printf("Debug: I2C bus address: %02X\n", i2c_port.addr);

   return(1);
   
}

// ------------------------------------------------------------
// Uses shtpData global -> todo change
// ------------------------------------------------------------
uint8_t bno_sendPacket(uint8_t channel, uint16_t datalen) {

   uint16_t packetlen = datalen;
   uint8_t *data;               // local buffer for I2C write data

   sequence[channel]++;         // increment seq for each packet
   packetlen = datalen + 4;     // Add four bytes for the header
   data = malloc(packetlen);    
   data[0] = packetlen & 0xFF;  // packet length LSB
   data[1] = packetlen >> 8;    // packet length MSB
   data[2] = channel;           // channel number
   data[3] = sequence[channel]; // packet sequence num

   // Copy the payload data from shtpData to the I2C data buffer
   for (short i = 0 ; i < datalen; i++) {
      data[4+i] = shtpData[i];
   }

   if(verbose >= 1) {
      printf("Debug: TX %3d bytes HEAD", packetlen);
      for (short i = 0 ; i < packetlen; i++) {
         if( i == 4 ) printf(" CARGO");
         printf(" %02X", data[i]);
      }
      printf("\n");
   }

   // --------------------------------------
   uint8_t rcode = send_data(data,packetlen);
   if ( rcode != 0 ) return(-1);

   return(1);
}

/* ------------------------------------------------------------ *
 * Check to see if there is any new data available. Read the    *
 * contents of the incoming packet into the shtpData array.     *
 * ------------------------------------------------------------ */
// ------------------------------------------------------------
uint16_t bno_readPacket(void) {

   int rbytes;                // Received bytes buffer
   uint8_t subtransfer = 0;   // if not all data is read in one go,
   
   clock_gettime(CLOCK_MONOTONIC, &t1);
   //t1.tv_sec = t2.tv_sec; t1.tv_nsec = t2.tv_nsec;
   // 1st Read to get the 4-byte SHTP header with the cargo size
   int cnt = 1;
   int cnt_err = 0;

   // Implement a busy wait pseudo block
   while ( 1 ) {

      // ------------------------------
      rbytes = read_data(shtpHeader, 4);

      if ( rbytes == 65535 ){
         printf("rbytes %d\n",rbytes);
         printf("%X %X %X %X\n",shtpHeader[0],shtpHeader[1],shtpHeader[2],shtpHeader[3]);
         usleep(I2CDELAY);
         continue;
      }

      if ( rbytes == -1 ) {
         usleep(5000);
         if ( cnt_err++ > 5 ) {
            printf("Error: Read Time out: %d.\n", cnt);
            return(0);
         }
         printf("Error: read %s %d\n",strerror(errno),errno);
         continue;
      }   
      
      if ( rbytes == 4 ) {

         // This is a busy wait. read does not block but return 0's for requsted number of bytes
         // Adjust the sleep duration depending on the acquisition time period. Making it about .8 of the
         // fastest report id seems to work best. 
         if ( shtpHeader[0] == 0 ) {
            usleep(1000 );  // grow wait time with each iteration to reduce cpu utilization
           
            cnt++;
            continue;         
         } 
         break; // break from while
      }   
      if ( rbytes < 4 ) {
         printf("Error: Partial Header Read: %d.\n", rbytes );
         printf("Error: %s\n", strerror(errno) );
         return(0);
      } 
     
      usleep(I2CDELAY);
   }

   //clock_gettime(CLOCK_MONOTONIC, &t2);
   //long diff = diff_ns(&t1,&t2); 
   //printf("\ntime diff => %ld\n",diff);

   // From the header bytes calculate the number of data bytes for the entire packet
   uint16_t packetlen = ((uint16_t) shtpHeader[1] << 8 | shtpHeader[0]);
   packetlen &= ~(1 << 15);       // Clear the MSbit.
   
   // Check if the subtransfer bit was set (shtpHeader[1] MSB)
   if(shtpHeader[1]&0x80) subtransfer = 1;

   // Found instances of noise rqst large packets. 
   if( packetlen > 512 ) {
      return(0);
   }

   uint8_t data[packetlen];    // Buffer for entire packet

   // --------------------------------
   rbytes = read_data(data, packetlen); 

   if( rbytes < packetlen ) {
      printf("Error: SHTP Cargo data read failure: got %d bytes should have %d\n", rbytes, packetlen);
      printf("Error: %s\n", strerror(errno));
      return(0);
   }
   // update the sequence counter for the channel
   sequence[data[2]] = data[3];
  
   // clear global data array, and write data buffer to it
   // gotta fix this and not use global 
   memset(shtpData, 0, sizeof shtpData);

   //if (rcnt++ % 1) { verbose = 0; }
   //else { verbose = 1; }
  
   if(verbose >= 2) printf("Debug: RX %3d bytes HEAD", packetlen);

   for (int i = 0; i < packetlen; i++) {
      if(verbose >= 2 ) {
         if ( i == 4 ) printf(" CARGO");
         if ( i < 31 ) printf(" %02X", data[i]);
         if ( i == 32 ) printf(" +%d more bytes", (packetlen-31));
      }
      if(i < 4) shtpHeader[i] = data[i]; // Store data into the shtpData array
      if(i > 3) shtpData[i-4] = data[i]; // Store data into the shtpData array
   }

   if(verbose >= 2) printf(" ST num [%d]\n", subtransfer);

   if(data[4] == COMMAND_RESPONSE) {
      printf("Debug: CMD reportID [%02X] REPseq [%02X] CMD [%02X] CMDseq [%02X] RESPseq [%02X] R0 [%02X]\n",
            data[4], data[5], data[6], data[7],data[8],data[9]);

            for(int i = 0; i<packetlen; i++) {
               printf("%02X ",data[i]);
            }
            printf("\n");
      return(0);      
   }

   return(packetlen);
  
}

// ------------------------------------------------------------
// Reset the sensor
// ------------------------------------------------------------
uint8_t bno_reset() {

   // Send "reset" command and read responses -------------
   shtpData[0] = 1;                   // CMD1 = reset
   uint8_t rtn = bno_sendPacket(CHANNEL_EXECUTABLE, 1); // Write command 1 byte to chan EXE
   if ( ! rtn ) { return(0); }
   sleep(1);

   // After reset 3 packets are sent back
   // Packet 1: advertising packet (chan 0)
   bno_readPacket();
   if( shtpHeader[2] != CHANNEL_COMMAND || shtpHeader[3] != 1) {
      printf("Error: can't get SHTP advertising.\n");
      return(0);
   }
   usleep(I2CDELAY);

    // Packet 2: "reset complete" (chan 1, response code 1)
   bno_readPacket();
   if(shtpHeader[2] != CHANNEL_EXECUTABLE || shtpHeader[3] != 1 || shtpData[0] != 1) {
      printf("Error: can't get 'reset complete' status.\n");
      return(0);
   }
   usleep(I2CDELAY);
    // Packet 3: SH-2 sensor hub SW init (chan 2)
   bno_readPacket();
   if(shtpHeader[2] != CHANNEL_CONTROL || shtpHeader[3] != 1) {
      printf("Error: can't get SH2 initialization.\n");
      return(0);
   }
   usleep(I2CDELAY);

   if(verbose >= 1) printf("Reset completed successfully!\n");

   return(1);
}

// ------------------------------------------------------------
uint8_t bno_errors() {
   // Test code: Below line simulates an SHTP error for incomplete
   // header data. SH2 will add the code 2 entry to the error list
   // sensor reset clears the error list.
   // char data[3] = { 2, 3, 4}; write(i2cfd, data, 3);

   /* --------------------------------------------------------- *
    * SHTP get error list from sensor                           *
    * --------------------------------------------------------- */
   shtpData[0] = 0x01;                // CMD 0x01 gets error list
   bno_sendPacket(CHANNEL_COMMAND, 1);    // Write 1 byte to chan 0
   usleep(I2CDELAY);                  

   /* --------------------------------------------------------- *
    * Get the SHTP error list, after reset it should be clean   *
    *  RX   5 bytes HEAD 05 80 00 03 CARGO 01 ST [0]            *
    * --------------------------------------------------------- */
   // Wait for answer packet
   uint16_t datalen;
   while ((datalen = bno_readPacket()) != 0) {
      
      if(shtpHeader[2] == CHANNEL_COMMAND && shtpData[0] == 0x01) break;
      usleep(I2CDELAY);
   }

   if(shtpHeader[2] != CHANNEL_COMMAND || shtpData[0] != 1) {
      printf("Error: can't get SHTP error list\n");
      return(0);
   }

   /* --------------------------------------------------------- *
    * Calculate the error counter                               *
    * --------------------------------------------------------- */
   int errcount = datalen - 1; // datalen minus 1 report byte

   return(errcount);
}

/* ------------------------------------------------------------ *
 *  
 * 
 * ------------------------------------------------------------ */
uint8_t bno_get_prod_id() {
   shtpData[0] = 0xF9;
   bno_sendPacket(CHANNEL_CONTROL, 2); 
   usleep(10000);

   uint16_t rtn = bno_readPacket();
   if ( rtn <= 0 ) { 
      printf("Set Feature failed %s\n",strerror(errno)); 
      bno_close();
      exit(1);
   }

   printf("\nSW Version\t %d.%d\n",shtpData[2],shtpData[3]);
   printf("SW Part No.\t %ul\n", (shtpData[7] << 24) | (shtpData[6] << 16) | (shtpData[5] << 8) | shtpData[4]);
   printf("Build No.\t %ul\n", (shtpData[11] << 24) | (shtpData[10] << 16) | (shtpData[9] << 8) | shtpData[8]);
	printf("SW Patch No.\t %d\n\n", (shtpData[13] << 8) | shtpData[12]);

   return(1);
}

/* ------------------------------------------------------------ *
 * Set Feature (ie get report at certain rate) 
 * SH-2 reference manual 6.5.4
 * ------------------------------------------------------------ */
uint8_t bno_set_feature(uint8_t r_id,int period_usec) {

   /* --------------------------------------------------------- *
    * Check if ACC is already enabled, if not enable it now...  *
    * --------------------------------------------------------- */
   shtpData[0] = SET_FEATURE_COMMAND;
   shtpData[1] = r_id;
   shtpData[5] = period_usec & 0xFF;
   shtpData[6] = (period_usec >> 8) & 0xFF;
   shtpData[7] = (period_usec >> 16) & 0xFF;
   shtpData[8] = (period_usec >> 24) & 0xFF;

   bno_sendPacket(CHANNEL_CONTROL, 17); 
   usleep(550000);                   // Needs a lot of time to get back... 

   uint16_t rtn = bno_readPacket();
   if ( rtn <= 0 ) { 
      printf("Set Feature failed %s\n",strerror(errno)); 
      bno_close();
      exit(1);
   }

   return(1);

   /*
   shtpData[0] = GET_FEATURE_REQUEST;
   shtpData[1] = 0x01;
   bno_sendPacket(CHANNEL_CONTROL, 2); // Write 20 bytes to CTL channel
   usleep(I2CDELAY);               // Delay 100 msecs before next I2C

   count = 0;
   datalen = 0;
   while ((datalen = bno_readPacket()) != 0) {
      if(count > 3) break;
      if(shtpHeader[2] == CHANNEL_CONTROL
         && shtpData[0] == GET_FEATURE_RESPONSE) break;
      usleep(I2CDELAY);             // Delay 100 msecs before next I2C
      count++;
   }

   if(shtpData[0] != GET_FEATURE_RESPONSE || 
      shtpData[1] != 0x01) {
      printf("Error: Not getting SHTP feature report\n");
      exit(-1);
   }
   if(verbose == 1) printf("Debug: FRS feature report received, [%d bytes]\n",
                            datalen);
   if(verbose == 1) printf("[%02X] [%02X] [%02X] [%02X]\n", shtpData[16],shtpData[17], shtpData[18],shtpData[19]);

   if(shtpData[16] == 0x00) return(1);
   usleep(200000);                   // wait 200 millisecs for completion
   datalen = bno_readPacket();
   */

   return(0);
}

/* ------------------------------------------------------------ *
 *    *
 *    *
 * ------------------------------------------------------------ */
uint8_t bno_read_event(struct State_t * state_p, uint8_t dsp_flg) {

   uint16_t rtn = bno_readPacket();
   if ( rtn <= 0 ) return 0; 

   // ^ Loads Global 
   // shtpHeader
   // shtpData

   // The Gyro Integrated Report is handled differently for efficiency evidently. 
   // This report is identifed if the channel in the header is set to 0x05
   // See 1.3.1 SHTP in BNO085 Data Sheet

   // The gyro-integrated input report is an odd ball require special handling. 
   // They are identified via the special gyro channel and do no include the 
   // usual ID, time stampe and status fields.

   // I hate special cases :(

   /* Overview of SHTP channels return in header shipHeader[2]:
   *
   * 0 -> Command
   * -- Used for protocol-global packets, currently only the advertisement packet (which lists all the channels) and error reports
   *
   * 1 -> Executable
   * -- Used for things that control the software on the chip: commands to reset and sleep
   * -- Also used by the chip to report when it's done booting up
   *
   * 2 -> Control
   * -- Used to send configuration commands to the IMU and for it to send back responses.
   * -- Common report IDs: Command Request (0xF2), Set Feature (0xFD)
   *
   * 3 -> Sensor Reports
   * -- Used for sensors to send back data reports.
   * -- AFAIK the only report ID on this channel will be 0xFB (Report Base Timestamp); sensor data is sent in a series of structures
   *    following an 0xFB
   *
   * 4 -> Wake Sensor/Normal Reports
   * -- same as above, but for sensors configured to wake the device
   *
   * 5 -> Gyro Rotation Vector
   * -- a dedicated channel for the Gyro Rotation Vector sensor report
   * -- Why does this get its own channel?  
   */

   if (shtpHeader[2] == 0x05 ) {
           
      // Give expected results... -----------------------------------------------
      state_p->quat[1] = (double)(int16_t)(shtpData[1] << 8 | shtpData[0]) * qp14;
      state_p->quat[2] = (double)(int16_t)(shtpData[3] << 8 | shtpData[2]) * qp14;
      state_p->quat[3] = (double)(int16_t)(shtpData[5] << 8 | shtpData[4]) * qp14;
      state_p->quat[0] = (double)(int16_t)(shtpData[7] << 8 | shtpData[6]) * qp14;
      
       state_p->omega[0] = (double)(int16_t)(shtpData[9] << 8  | shtpData[8]) * qp10 * rad2deg;
      state_p->omega[1] = (double)(int16_t)(shtpData[11] << 8 | shtpData[10]) * qp10 * rad2deg;
      state_p->omega[2] = (double)(int16_t)(shtpData[13] << 8 | shtpData[12]) * qp10 * rad2deg;

      state_p->fs = 0x2A;
      //--------------------------------------------------------------------------------------


      // As given in BNO080.cpp doesn't work. --------------
      /*
      uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;

      rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
      rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
      rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

      state_p->omega[0] = (double)( rawFastGyroX ) * qp10 * rad2deg;
      state_p->omega[1] = (double)( rawFastGyroY ) * qp10 * rad2deg;
      state_p->omega[2] = (double)( rawFastGyroZ ) * qp10 * rad2deg;
      */
      // printf("%7.3f \n",state_p->omega[2]);

      quat2euler( state_p->quat, state_p->angle );

      if ( dsp_flg ) disp_rpy_omega(state_p);

      return 1; 
   } 

   // The other reports are structured as follows using report id 0x01: 
   //
   //shtpHeader[0:3]: A 4 byte header containing packet length, subtransfer bit, channel
   //shtpData[0:4]: A 5 byte timestamp of microsecond since reading was taken
   //shtpData[5 + 0]: A feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
   //shtpData[5 + 1]: Sequence number (See 6.5.18.2)
   //shtpData[5 + 2]: Status
   //shtpData[3]: Delay
   //shtpData[4:5]: x/accel x/gyro x/etc
   //shtpData[6:7]: y/accel y/gyro y/etc
   //shtpData[8:9]: z/accel z/gyro z/etc
   //shtpData[10:11]: real/gyro temp/etc
   //shtpData[12:13]: Accuracy estimate
   // --------------------------------
   // Depending on the request rate one read can have two samples
   // size of report SIZE
   // --------------------------------
   //shtpData[SIZE + 0]: A feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
   //shtpData[SIZE + 1]: Sequence number (See 6.5.18.2)
   //shtpData[SIZE + 2]: Status
   //shtpData[SIZE + 3]: Delay
   //shtpData[SIZE + 4:5]: x/accel x/gyro x/etc
   //shtpData[SIZE + 6:7]: y/accel y/gyro y/etc
   //shtpData[SIZE + 8:9]: z/accel z/gyro z/etc
   //shtpData[SIZE + 10:11]: real/gyro temp/etc
   //shtpData[SIZE + 12:13]: Accuracy estimate

   int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
   dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.

   state_p->ts = ((uint32_t)shtpData[4] << 24) | ((uint32_t)shtpData[3] << 16) | ((uint32_t)shtpData[2] << 8) | ((uint32_t)shtpData[1] );

   // lots of reports use 3 16-bit numbers stored in bytes 4 through 9
   int16_t data1 = (int16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
   int16_t data2 = (int16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
   int16_t data3 = (int16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];

   //uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits comment as not used yet
   
   uint16_t data4; 
   uint16_t data5; 
   if (dataLength - 5 > 9)
   {
      data4 = (uint16_t)shtpData[16] << 8 | shtpData[15];
   }

   if (dataLength - 5 > 11)
   {
      data5 = (uint16_t)shtpData[18] << 8 | shtpData[17];
   }

   // Now look for report id type
   // Accel (with gravity vector) m/s^2
   if (shtpData[5] == 0x01)
   {
      state_p->acc[0] = (double)data1 * qp8;
      state_p->acc[1] = (double)data2 * qp8;
      state_p->acc[2] = (double)data3 * qp8;

      if ( dsp_flg ) {
           printf("%7.4f %7.4f %7.4f\n",state_p->acc[0], state_p->acc[1], state_p->acc[2]);
      }

   }
   else if ( shtpData[5] == 0x05 || shtpData[5] == 0x08 || shtpData[5] == 0x28 || shtpData[5] == 0x29 )
   {
      state_p->quat[1] = (double)data1 * qp14;
      state_p->quat[2] = (double)data2 * qp14;
      state_p->quat[3] = (double)data3 * qp14;
      state_p->quat[0] = (double)data4 * qp14;

      quat2euler( state_p->quat, state_p->angle );
   }
   else {
      // Lots more needs to added here along with different destination structures. 
      // For my purposes not using these... if anyone adds please send patch. 
      printf("unhandled report id\n");
   }

   
   return 1; 
}

// -----------------------------------------
// Display functions. 
void disp_rpy_omega(struct State_t * state_p){
   printf("%7.4f %7.4f %7.4f : ",state_p->angle[0], state_p->angle[1], state_p->angle[2]);
   printf("%7.4f %7.4f %7.4f",state_p->omega[0], state_p->omega[1], state_p->omega[2]);
   printf("\n");
}

void disp_three_floats(struct State_t * state_p){
   printf("%7.4f %7.4f %7.4f\n",state_p->angle[0], state_p->angle[1], state_p->angle[2]);
}

//-------------------------------------------
//-------------------------------------------
// ---- Hardware Specific API Functions -----

//-------------------------------------------
uint8_t send_i2c_bcm(uint8_t *data,uint16_t data_len){

   uint8_t rcode = bcm2835_i2c_write((char *)data,data_len);

   if ( rcode != BCM2835_I2C_REASON_OK ) {
      if ( rcode == BCM2835_I2C_REASON_ERROR_NACK) perror("Received a NACK on send");
      if ( rcode == BCM2835_I2C_REASON_ERROR_CLKT) perror("Received Clock Stretch Timeout");
      if ( rcode == BCM2835_I2C_REASON_ERROR_DATA) perror("Not all data received");
      return(-1);
   }

   return(data_len);
}

//------------------------------------------
uint8_t send_i2c_dev(uint8_t *data,uint16_t data_len){

   int rtn = write(i2c_port.fd, data, data_len);

   if( rtn != data_len ) {
      printf("Error: Write failed on %d tried to send %d only %d bytes sent %s\n", i2c_port.fd,data_len,rtn,strerror(errno));
      return(-1);
   }

   return(rtn);
}

//-------------------------------------------
uint16_t read_i2c_bcm(uint8_t *data,uint16_t data_len){

   int rcode = bcm2835_i2c_read((char *)data, data_len);

   if ( rcode != BCM2835_I2C_REASON_OK ) {
      if ( rcode == BCM2835_I2C_REASON_ERROR_NACK) perror("Received a NACK on read");
      if ( rcode == BCM2835_I2C_REASON_ERROR_CLKT) perror("Received Clock Stretch Timeout");
      if ( rcode == BCM2835_I2C_REASON_ERROR_DATA) perror("Not all data received");
      return(-1);
   }

   // If no error code was returned assume the bytes requested was received. 
   return(data_len);
}

//--------------------------------------------
uint16_t read_i2c_dev(uint8_t *data,uint16_t data_len){

   int rtn = read(i2c_port.fd, data, data_len);

   if( rtn != data_len ) {
      printf("Error: Read failed on %s requested %d only %d bytes sent %s\n", i2c_port.dev,data_len,rtn,strerror(errno));
      bno_close();
      usleep(10000);
      bno_open();
      usleep(10000);
      
      return(-1);
   }

   return(rtn);
}

// ----------------------------
void close_i2c_bcm(){
   bcm2835_i2c_end();
}

// ----------------------------
void close_i2c_dev(){
   close(i2c_port.fd);
}

// --- Util Functions --- //
uint8_t quat2euler(double * qt, double * angles) {
   
   // ref. // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qt[0] * qt[1] + qt[2] * qt[3]);
    double cosr_cosp = 1 - 2 * (qt[1] * qt[1] + qt[2] * qt[2]);
    angles[0] = atan2(sinr_cosp, cosr_cosp) * rad2deg;

    // pitch (y-axis rotation)
    double sinp = 2 * (qt[0] * qt[2] - qt[3] * qt[1]);
    if (abs(sinp) >= 1)
        angles[1] = copysign(M_PI / 2, sinp) * rad2deg; // use 90 degrees if out of range
    else
        angles[1] = asin(sinp) * rad2deg;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qt[0] * qt[3] + qt[1] * qt[2]);
    double cosy_cosp = 1 - 2 * (qt[2] * qt[2] + qt[3] * qt[3]);
    angles[2] = atan2(siny_cosp, cosy_cosp) * rad2deg;

    return 1;
}

// --- Timing utility function ----
long diff_ns(struct timespec * t1, struct timespec * t2)
{
   // t1 -> prev
   // t2 -> now
   long tdiff; 
   if ( t1->tv_sec == t2->tv_sec ){
      tdiff = t2->tv_nsec - t1->tv_nsec; 
   } 
   else {
      tdiff = (t2->tv_sec - t1->tv_sec) * 1000000000UL + (t2->tv_nsec - t1->tv_nsec); 
   }
   return(tdiff);
}	
