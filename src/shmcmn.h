#ifndef SHMCMN_H
#define SHMCMN_H

#define SHMFILE  "/dev/shm/gyro0"


void * open_shm(char * shmfile,unsigned int memsize);
void close_shm(void * state_ptr,unsigned int memsize);

long diff_ns(struct timespec * t1, struct timespec * t2);

// ---- Error Macro ----
#define PrintErr(...) do { \
    fprintf (stderr,__VA_ARGS__); \
    fprintf (stderr," @ %s (%d)\n", __FILE__, __LINE__ - 2); \
} while (0)


#endif /* SHMCMN_H */