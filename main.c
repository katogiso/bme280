#include "utils.h"
#include "i2c.h"

#define BME280_DATA_INDEX    1
#define BME280_ADDRESS_INDEX 2
#define I2C_BUFFER_LEN 8

s8  set_I2C_routines(void);
s8  BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8  BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s32 bme280_data_readout(void);

struct bme280_t bme280;

int main(){

  set_I2C_routines();

  return bme280_data_readout();
}

s32 bme280_data_readout(void)
{

  u8  stand_by_time_u8 = 0;
  s32 data_uncomp_temp = 0;
  s32 data_uncomp_pres = 0;
  s32 data_uncomp_hum  = 0;
  s32 comp_temp        = 0;
  u32 comp_press       = 0;
  u32 comp_humidity    = 0;
  s32 status           = ERROR;

  int i;

  // To get the calibration parameters
  status  = bme280_initialize(&bme280); 

  // To select the bme280 modes
  status += bme280_set_power_mode( &bme280, BME280_FORCED_MODE);

  // To select the bme280 sampling mode
  status += bme280_set_oversamp_temperature(&bme280, BME280_OVERSAMP_4X);
  status += bme280_set_oversamp_pressure(&bme280, BME280_OVERSAMP_2X);
  status += bme280_set_oversamp_humidity(&bme280, BME280_OVERSAMP_1X);

  // To select the bme280 sampling mode
  status += bme280_set_standby_durn(&bme280, BME280_STANDBY_TIME_1_MS);
  status += bme280_get_standby_durn(&bme280, &stand_by_time_u8);

  // To read the data of raw data from Registers.
  status += bme280_read_uncomp_temperature(&bme280, &data_uncomp_temp);
  status += bme280_read_uncomp_pressure(&bme280, &data_uncomp_pres);
  status += bme280_read_uncomp_humidity(&bme280, &data_uncomp_hum);

  // To convert the data to human readable values.
  comp_temp     = bme280_compensate_temperature_int32(&bme280, data_uncomp_temp);
  comp_press    = bme280_compensate_pressure_int32(&bme280, data_uncomp_pres);
  comp_humidity = bme280_compensate_humidity_int32(&bme280, data_uncomp_hum);

  status += bme280_set_power_mode(&bme280, BME280_SLEEP_MODE);

  printf( "%f,", comp_temp / 100.0 );
  printf( "%f,", comp_press /100.0 );
  printf( "%f\n", comp_humidity / 1024.0 );

  return status;
}


s8 set_I2C_routines(void) {
  bme280.bus_write  = BME280_I2C_bus_write;
  bme280.bus_read   = BME280_I2C_bus_read;
  bme280.dev_addr   = BME280_I2C_ADDRESS1;
  bme280.delay_msec = delay_msec;
  return BME280_INIT_VALUE;
}

// In the driver SUCCESS defined as 0 and FAILURE defined as -1 
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  s32 status    = 0;
  u8  stringpos = 0;
  u8  array[I2C_BUFFER_LEN];
  int fd;

  array[BME280_INIT_VALUE] = reg_addr;

  for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
    array[stringpos + BME280_DATA_INDEX] = *(reg_data + stringpos);
  }

  status  = i2c_open( &fd, 1, dev_addr );
  status += i2c_write( &fd, array, cnt+1 );
  status += i2c_close( &fd );

  return (s8)status;
}

// In the driver SUCCESS defined as 0 and FAILURE defined as -1 
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  s32 status   = 0;
  u8  stringpos;
  u8  array[I2C_BUFFER_LEN];
  int fd;

  status  = i2c_open( &fd, 1, dev_addr );
  status += i2c_read( &fd, reg_addr, array, cnt );

  for( stringpos = 0; stringpos < cnt; stringpos++ ) {
    *(reg_data + stringpos) = array[stringpos];
  }

  status += i2c_close( &fd );

  return (s8)status;
}


