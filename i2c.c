#include "i2c.h"

int i2c_open( int *fd, int bus, unsigned char dev_addr ) 
{
  char i2c_dev_fn[64];

  sprintf(i2c_dev_fn, "/dev/i2c-%d", bus);

  if (( (*fd) = open(i2c_dev_fn, O_RDWR)) < 0) {
    return -1;
  }

  if (ioctl( (*fd), I2C_SLAVE, dev_addr ) < 0) {
    return -2;
  }

  return 0;
}

int i2c_close( int *fd )
{
  close( *fd );
  return 0;
}


int i2c_read( int *fd, unsigned char reg, unsigned char *dat,  unsigned char cnt ) 
{

  /* write address */
  if ((write(*fd, &reg, 1)) != 1) { 
    return -3;
  }

  /* read data */
  if (read(*fd, dat, cnt) != cnt ) { 
    return -4;
  }

  return 0;
}


int i2c_write( int *fd, unsigned char *dat, unsigned char cnt ) 
{
  
  /* write address */
  if ((write(*fd, dat, cnt)) != cnt) { 
    return -3;
  }

  return 0;
}

void delay_msec( u32 msec )
{

  struct timespec ts;
  
  ts.tv_sec = 0;
  ts.tv_nsec = msec * 1000 * 1000;
  
  nanosleep( &ts, NULL );

  return;
}
