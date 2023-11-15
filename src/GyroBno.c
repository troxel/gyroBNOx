#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

// Shared mem stuff
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>

// Order is important... 
#include "mstate.h"
#include "gyrobno.h"
#include "apibno.h"
#include "shmcmn.h"

struct State_t mstate;	
struct State_t *mstate_ptr;

void intHandler(int sig);

uint8_t (*bno_open)() = NULL;
void (*bno_close)() = NULL;
uint8_t verbose =1;

#define MAX_INPUT_LENGTH 15

// -----------------------------------------------
int main(int argc, char **argv) {
   
	signal(SIGINT, intHandler);
	signal(SIGTSTP, intHandler);

   char dev_i2c[MAX_INPUT_LENGTH];
	dev_i2c[0] = '\0'; 

   unsigned int feature_set = 0x2A;
   int interval = 0;
   int opt;

    while ((opt = getopt(argc, argv, "d:f:i:")) != -1) {
		  printf("opt is %C",opt);
        switch (opt) {
            case 'd':
					// Protect buffer
				  	if (strlen(optarg) > MAX_INPUT_LENGTH) {
                  fprintf(stderr, "Error: Arg too long. Max length is %d characters.\n", MAX_INPUT_LENGTH);
                  return EXIT_FAILURE;
               }
					strncpy(dev_i2c,optarg, sizeof(dev_i2c));

               break;

            case 'f':
                sscanf(optarg, "%x", &feature_set);
                break;
            case 'i':
                interval = atoi(optarg);
                break;
            default:
                fprintf(stderr, "\nUsage: %s [-d i2c-device] [-f feature-set-hex] [-i report interval]\n", argv[0]);
                fprintf(stderr, "-d : Must be either bcm2835 or existinbg i2c device in /dev\n");
                fprintf(stderr, "-f : Feature set hex number of ST-2 manual\n");
                fprintf(stderr, "-i : Interval for printing results to console.\n\n");
                exit(EXIT_FAILURE);
        }
    }

	 if ( dev_i2c[0] == '\0' ) {
		   fprintf(stderr, "I2C Device -d is required\n");
			fprintf(stderr,"Example %s -d /dev/i2c-1 or %s -d bcm2835\n\n",argv[0],argv[0]);
         exit(EXIT_FAILURE);
	 }

    printf("I2C Device: %s\n", dev_i2c);
    printf("Feature Set: 0x%X\n", feature_set);
    printf("Report Interval: %d\n", interval);


   //struct timespec t1={0,0}, t2={0,0};

	mstate_ptr = open_shm(SHMFILE,sizeof(mstate));

	bno_init(dev_i2c);

   uint8_t open_rtn = bno_open();
   if ( open_rtn <= 0 ) { 
		printf("cannot open port %s\n",strerror(errno)); 
		bno_close();
		return(-1); 
	}

  	uint8_t rtn = bno_reset();
    if ( rtn <= 0 ) { 
		printf("Reset failed %s\n",strerror(errno)); 
		bno_close();
		return(-1); 
	}

	// -------------------------------------------------------------------------------------
	// Modify below as desired... 
	// Report ID 0x2A is integrated gyro produces both rotation vector and ang velocity... just what we want 
	// 5000 -> 200 hz
	// 2500 -> 400
	// 1250 -> 800
	//bno_set_feature(0x2A,1250);
	bno_set_feature(0x01,1250);
	//bno_set_feature(0x05,5000);
	//bno_set_feature(0x06,1250);

	uint16_t cnt=0; 
	uint16_t dsp_rate = 100;
	uint8_t dsp_flg = 0; 
	//clock_gettime(CLOCK_MONOTONIC, &t1);

	usleep(200);
	while(1){

		dsp_flg = 0;
		if ( cnt++ % dsp_rate == 0  ) dsp_flg = 1;

		uint8_t rtn = bno_read_event(mstate_ptr,dsp_flg);
		if ( rtn == 0 ) continue; 

		//printf("%5.2f %5.2f %5.2f %5.2f\n",mstate.quat[0],mstate.quat[1],mstate.quat[2],mstate.quat[3]);
	}
}	

void  intHandler(int sig)
{
	printf("Caught signal closing... \n");
	uint16_t errnum = bno_errors();
	printf("error num %d\n",errnum);
	bno_close();
	close_shm(mstate_ptr,sizeof(mstate));
	exit(1);
}






