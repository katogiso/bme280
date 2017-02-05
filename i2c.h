#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <time.h>
#include <sys/time.h>

#include "utils.h"

int i2c_open( int *fd, int bus, unsigned char dev_addr );
int i2c_write( int *fd, unsigned char *dat, unsigned char cnt ) ;
int i2c_read( int *fd, unsigned char reg, unsigned char *dat,  unsigned char cnt ) ;
void delay_msec( u32 msec );
