#include "utils.h"
#include "i2c.h"

#define BME280_DATA_INDEX    1
#define BME280_ADDRESS_INDEX 2

#define I2C_BUFFER_LEN 8

struct bme280_t bme280;

int main(){
  bme280_data_readout_template();
}

// In the driver SUCCESS defined as 0 and FAILURE defined as -1 
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError    = BME280_INIT_VALUE;
    u8  stringpos = BME280_INIT_VALUE;
    u8  array[I2C_BUFFER_LEN];
    int fd;

    array[BME280_INIT_VALUE] = reg_addr;

    for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos + BME280_DATA_INDEX] = *(reg_data + stringpos);
    }

    iError += i2c_open( &fd, 1, dev_addr );
    i2c_write( &fd, array, cnt+1 );
    i2c_close( &fd, 1, dev_addr );

    return (s8)iError;
}

// In the driver SUCCESS defined as 0 and FAILURE defined as -1 
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError    = BME280_INIT_VALUE;
    u8  stringpos;
    u8  array[I2C_BUFFER_LEN];
    int fd;

    iError += i2c_open( &fd, 1, dev_addr );
    iError += i2c_read( &fd, reg_addr, array, cnt );

    for( stringpos = 0; stringpos < cnt; stringpos++ ) {
        *(reg_data + stringpos) = array[stringpos];
    }

    i2c_close( &fd );

    return (s8)iError;
}

s8 I2C_routine(void) {
    bme280.bus_write  = BME280_I2C_bus_write;
    bme280.bus_read   = BME280_I2C_bus_read;
    bme280.dev_addr   = BME280_I2C_ADDRESS1;
    bme280.delay_msec = delay_msec;
    return BME280_INIT_VALUE;
}


s32 bme280_data_readout_template(void)
{

    u8  v_stand_by_time_u8     = BME280_INIT_VALUE;
    s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
    s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
    s32 v_data_uncomp_hum_s32  = BME280_INIT_VALUE;
    s32 v_comp_temp_s32[2]     = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_press_u32[2]    = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    s32 com_rslt               = ERROR;

    int i;

    I2C_routine();

    com_rslt  = bme280_initialize(&bme280); // To get the calibration data

    com_rslt += bme280_set_power_mode( &bme280, BME280_FORCED_MODE);

    com_rslt += bme280_set_oversamp_temperature(&bme280, BME280_OVERSAMP_4X);
    com_rslt += bme280_set_oversamp_pressure(&bme280, BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(&bme280, BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(&bme280, BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_get_standby_durn(&bme280, &v_stand_by_time_u8);

    com_rslt += bme280_read_uncomp_temperature(&bme280, &v_data_uncomp_temp_s32);
    com_rslt += bme280_read_uncomp_pressure(&bme280, &v_data_uncomp_pres_s32);
    com_rslt += bme280_read_uncomp_humidity(&bme280, &v_data_uncomp_hum_s32);

    v_comp_temp_s32[0]     = bme280_compensate_temperature_int32(&bme280, v_data_uncomp_temp_s32);
    v_comp_press_u32[0]    = bme280_compensate_pressure_int32(&bme280, v_data_uncomp_pres_s32);
    v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(&bme280, v_data_uncomp_hum_s32);


    // com_rslt += bme280_read_pressure_temperature_humidity( &v_comp_press_u32[1], 
    // 							   &v_comp_temp_s32[1], 
    // 							   &v_comp_humidity_u32[1]);

    com_rslt += bme280_set_power_mode(&bme280, BME280_SLEEP_MODE);


    for( i=0; i < 1; i++ ){
      printf( "%f,", v_comp_temp_s32[i] / 100.0 );
      printf( "%f,", v_comp_press_u32[i] /100.0 );
      printf( "%f\n", v_comp_humidity_u32[i] / 1024.0 );
    }

    return com_rslt;
}


