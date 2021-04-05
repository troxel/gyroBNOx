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

// -----------------------------------------------
int main(int argc, char **argv) {
	
	signal(SIGINT, intHandler);
	signal(SIGTSTP, intHandler);

    //struct timespec t1={0,0}, t2={0,0};

	//mstate_ptr = &(mstate);
	mstate_ptr = open_shm(SHMFILE,sizeof(mstate));

	printf("start\n");
	
	bno_init(USEI2CBCM); // Select comm method... uses function pointers 

    if ( bno_open() <= 0 ) { 
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
	bno_set_feature(0x2A,1250);

	int cnt=0; 
	//clock_gettime(CLOCK_MONOTONIC, &t1);

	usleep(200);
	while(1){

		uint8_t rtn = bno_read_event(mstate_ptr);

		if ( cnt % 100 == 0  ) {
			printf("%7.2f %7.2f %7.2f : ",mstate_ptr->angle[0], mstate_ptr->angle[1], mstate_ptr->angle[2]);
			printf("%7.2f %7.2f %7.2f",mstate_ptr->omega[0], mstate_ptr->omega[1], mstate_ptr->omega[2]);
			printf("\n");
			//printf("%5.2f %5.2f %5.2f %5.2f\n",mstate.quat[0],mstate.quat[1],mstate.quat[2],mstate.quat[3]);
		}

		if ( ! rtn ) {
			printf("rtn=%d",rtn); 
			continue; 
		}

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






