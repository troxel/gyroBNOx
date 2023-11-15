#ifndef MSTATE_H
#define MSTATE_H

struct State_t {
   double angle[3];
	double omega[3];
	double acc[3];  // x,y,z
	double quat[4]; // real,i,j,k
	double tdata[3];
	long ts;
	uint8_t fs; 
};

//uint8_t shtpHeaderRcv[4]; 
//uint8_t shtpDataRcv[256]; 

#endif /* MSTATE_H */