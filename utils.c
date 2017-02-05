#include "utils.h"

s8 bme280_get_reg_value( struct bme280_t *p_bme280, u8 *value, u8 addr, u8 msk, u8 pos )
{
  s8 status = ERROR;
  u8 reg_data;

  u8 dev_addr    = p_bme280->dev_addr;
  BME280_RD_FUNC = p_bme280->bus_read;

  status = read( dev_addr, addr, &reg_data, 1);
  *value = ((reg_data & msk ) >> pos );

  return status;
}

s8 bme280_id_read(struct bme280_t *p_bme280)
{
  
  bme280_get_reg_value( p_bme280, 
                        &(p_bme280->chip_id), 
                        BME280_CHIP_ID_REG, 
                        0xF, 
                        0 );

  return ( ( p_bme280->chip_id == BME280_CHIP_ID) ? SUCCESS : ERROR);
}

s8 bme280_get_calib_param(struct bme280_t *p_bme280)
{
  BME280_RD_FUNC = p_bme280->bus_read;
  u8 dev_addr    = p_bme280->dev_addr;
  s8 status      = ERROR;

  u8 data[BME280_CALIB_DATA_SIZE];
  s8 i,j;
  u8 step = 2;
  u8 tmp[2];

  for( i = 0 ; i <  BME280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH ; i=i+2 ){
    status = read( dev_addr, BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG + i , tmp, step );
    for( j = 0 ; j < step ; j++ ){
      data[i+j] = tmp[j];
    }
  }

  p_bme280->cal_param.dig_T1 = BME280_CAST_TO_U16(data, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T1_MSB, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T1_LSB);
  p_bme280->cal_param.dig_T2 = BME280_CAST_TO_S16(data, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T2_MSB, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T2_LSB);
  p_bme280->cal_param.dig_T3 = BME280_CAST_TO_S16(data, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T3_MSB, 
                                                  BME280_TEMPERATURE_CALIB_DIG_T3_LSB);
  p_bme280->cal_param.dig_P1 = BME280_CAST_TO_U16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P1_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P1_LSB);
  p_bme280->cal_param.dig_P2 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P2_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P2_LSB);
  p_bme280->cal_param.dig_P3 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P3_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P3_LSB);
  p_bme280->cal_param.dig_P4 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P4_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P4_LSB);
  p_bme280->cal_param.dig_P5 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P5_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P5_LSB);
  p_bme280->cal_param.dig_P6 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P6_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P6_LSB);
  p_bme280->cal_param.dig_P7 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P7_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P7_LSB);
  p_bme280->cal_param.dig_P8 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P8_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P8_LSB);
  p_bme280->cal_param.dig_P9 = BME280_CAST_TO_S16(data, 
                                                  BME280_PRESSURE_CALIB_DIG_P9_MSB   , 
                                                  BME280_PRESSURE_CALIB_DIG_P9_LSB);
  p_bme280->cal_param.dig_H1 = data[BME280_HUMIDITY_CALIB_DIG_H1];
  
  status += p_bme280->bus_read(dev_addr, 
                               BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG, 
                               data, 
                               BME280_HUMIDITY_CALIB_DATA_LENGTH);

  p_bme280->cal_param.dig_H2 = BME280_CAST_TO_S16(data, 
                                                  BME280_HUMIDITY_CALIB_DIG_H2_MSB    , 
                                                  BME280_HUMIDITY_CALIB_DIG_H2_LSB);
  p_bme280->cal_param.dig_H3 = data[BME280_HUMIDITY_CALIB_DIG_H3];

  p_bme280->cal_param.dig_H4 = (s16)((((s16)((s8)data[BME280_HUMIDITY_CALIB_DIG_H4_MSB])) << 
                                      BME280_SHIFT_BIT_POSITION_BY_04_BITS) |
                                     (((u8)BME280_MASK_DIG_H4) & data[BME280_HUMIDITY_CALIB_DIG_H4_LSB]));

  p_bme280->cal_param.dig_H5 = (s16)((((s16)((s8)data[BME280_HUMIDITY_CALIB_DIG_H5_MSB])) << 
                                      BME280_SHIFT_BIT_POSITION_BY_04_BITS) | 
                                     (data[BME280_HUMIDITY_CALIB_DIG_H4_LSB] >> 
                                      BME280_SHIFT_BIT_POSITION_BY_04_BITS));

  p_bme280->cal_param.dig_H6 = (s8)data[BME280_HUMIDITY_CALIB_DIG_H6];

  return status;
}



s8 bme280_get_power_mode( struct bme280_t *p_bme280, u8 *power_mode )
{
  s8 status = ERROR;
  u8 dev_addr = p_bme280->dev_addr;


  status = bme280_get_reg_value( p_bme280, power_mode, 
                                 BME280_CTRL_MEAS_REG, 
                                 BME280_CTRL_MEAS_REG_POWER_MODE__MSK, 
                                 BME280_CTRL_MEAS_REG_POWER_MODE__POS );

  return status;
}

s8 bme280_set_power_mode( struct bme280_t *p_bme280, u8 power_mode )
{
  s8 status;
  u8 ctrl_meas;
  u8 pre_power_mode;
  u8 dev_addr = p_bme280->dev_addr;

  BME280_RD_FUNC = p_bme280->bus_read;
  BME280_WR_FUNC = p_bme280->bus_write;

  status = bme280_get_power_mode(p_bme280, &pre_power_mode);

  if( pre_power_mode == power_mode ){
    // nop
  } else {
    status    = read( dev_addr, BME280_CTRL_MEAS_REG, &ctrl_meas, 1 );
    ctrl_meas = BME280_SET_BITSLICE(ctrl_meas, BME280_CTRL_MEAS_REG_POWER_MODE, power_mode);
    write( dev_addr, BME280_CTRL_MEAS_REG, &ctrl_meas, 1 );
  }

  return status;
}

s8 bme280_set_oversamp_temperature( struct bme280_t *p_bme280, u8 mode )
{
  s8 status   = ERROR;
  u8 data     = 0;
  u8 dev_addr = p_bme280->dev_addr;

  u8 v_pre_ctrl_hum_value_u8 = BME280_INIT_VALUE;
  u8 v_pre_config_value_u8   = BME280_INIT_VALUE;
  BME280_WR_FUNC             = p_bme280->bus_write;


  status  = bme280_get_reg_value( p_bme280, &data, BME280_CTRL_MEAS_REG, 0xF, 0 );
  data    = BME280_SET_BITSLICE(data, BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE, mode);
  status += write( dev_addr, BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG, &data, 1);

  p_bme280->oversamp_temperature = mode;

  status += bme280_get_reg_value( p_bme280, &data, 
                                  BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG, 
                                  BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK,
                                  BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS);
  
  return status;
}

s8 bme280_set_oversamp_pressure( struct bme280_t *p_bme280, u8 mode )
{
  s8 status   = ERROR;
  u8 data     = 0;
  u8 dev_addr = p_bme280->dev_addr;

  u8 v_pre_ctrl_hum_value_u8 = BME280_INIT_VALUE;
  u8 v_pre_config_value_u8   = BME280_INIT_VALUE;
  BME280_WR_FUNC             = p_bme280->bus_write;


  status  = bme280_get_reg_value( p_bme280, &data, BME280_CTRL_MEAS_REG, 0xF, 0 );
  data    = BME280_SET_BITSLICE(data, BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE, mode);
  status += write( dev_addr, BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG, &data, 1);

  p_bme280->oversamp_pressure = mode;

  status += bme280_get_reg_value( p_bme280, &data, 
                                  BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG, 
                                  BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK,
                                  BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS);
  
  return status;
}

s8 bme280_set_oversamp_humidity( struct bme280_t *p_bme280, u8 mode )
{
  s8 status   = ERROR;
  u8 data     = 0;
  u8 dev_addr = p_bme280->dev_addr;

  BME280_WR_FUNC = p_bme280->bus_write;

  status  = bme280_get_reg_value( p_bme280, &data, BME280_CTRL_HUMIDITY_REG, 0xF, 0 );
  data    = BME280_SET_BITSLICE(data, BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY, mode);
  status += write( dev_addr, BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__REG, &data, 1);

  p_bme280->oversamp_pressure = mode;

  status += bme280_get_reg_value( p_bme280, &data, 
                                  BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__REG, 
                                  BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__MSK,
                                  BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__POS);
  return status;
}


s8 bme280_read_uncomp_pressure( struct bme280_t *p_bme280, s32 *uncomp_pressure )
{
  s8 status = ERROR;
  u8 data[BME280_PRESSURE_DATA_SIZE] = { 0, 0, 0};

  u8 dev_addr    = p_bme280->dev_addr;
  BME280_RD_FUNC = p_bme280->bus_read;

  status = read( dev_addr,BME280_PRESSURE_MSB_REG, data, BME280_PRESSURE_DATA_LENGTH);

  *uncomp_pressure = (s32)(((u32)(data[BME280_PRESSURE_MSB_DATA])  << 12 ) );
  *uncomp_pressure = (s32)(((u32)(data[BME280_PRESSURE_LSB_DATA])  <<  4 ) | *uncomp_pressure );
  *uncomp_pressure = (s32)(((u32)(data[BME280_PRESSURE_XLSB_DATA]) >>  4 ) | *uncomp_pressure );

  return status;
}

s8 bme280_read_uncomp_temperature( struct bme280_t *p_bme280, s32 *uncomp_temperature)
{

  s8 status = ERROR;
  u8 data[BME280_TEMPERATURE_DATA_SIZE] = { 0,0,0};

  u8 dev_addr    = p_bme280->dev_addr;
  BME280_RD_FUNC = p_bme280->bus_read;


  status = read( dev_addr, BME280_TEMPERATURE_MSB_REG, data, BME280_TEMPERATURE_DATA_LENGTH);
  *uncomp_temperature = (s32)(((u32)(data[BME280_TEMPERATURE_MSB_DATA])  << 12) );
  *uncomp_temperature = (s32)(((u32)(data[BME280_TEMPERATURE_LSB_DATA])  << 4 ) | *uncomp_temperature);
  *uncomp_temperature = (s32)(((u32)(data[BME280_TEMPERATURE_XLSB_DATA]) >> 4 ) | *uncomp_temperature);
  return status;
}

s8 bme280_read_uncomp_humidity( struct bme280_t *p_bme280, s32 *uncomp_humidity)
{

  s8 status = ERROR;
  u8 data[BME280_HUMIDITY_DATA_SIZE] = { 0, 0 };

  u8 dev_addr    = p_bme280->dev_addr;
  BME280_RD_FUNC = p_bme280->bus_read;

  status = read( dev_addr, BME280_HUMIDITY_MSB_REG, data, BME280_HUMIDITY_DATA_LENGTH);
  *uncomp_humidity = (s32)((u32)data[BME280_HUMIDITY_MSB_DATA] << 8 );
  *uncomp_humidity = (s32)((u32)(data[BME280_HUMIDITY_LSB_DATA]) | *uncomp_humidity );

  return status;
}

s32 bme280_compensate_temperature_int32( struct bme280_t *p_bme280, s32 uncomp_temperature)
{
  s32 x1 = BME280_INIT_VALUE;
  s32 x2 = BME280_INIT_VALUE;
  s32 temperature = BME280_INIT_VALUE;

  u16 dig_T1 = p_bme280->cal_param.dig_T1;
  s16 dig_T2 = p_bme280->cal_param.dig_T2;
  s16 dig_T3 = p_bme280->cal_param.dig_T3;

  x1 = (((uncomp_temperature >> 3) - ((s32)dig_T1<<1))*((s32)dig_T2));
  x1 = x1 >> 11;
  x2 = (uncomp_temperature >> 4) - ((s32)dig_T1);
  x2 = x2 * x2;
  x2 = x2 >> 12;
  x2 = x2 >> dig_T3;
  x2 = x2 >> 14;

  p_bme280->cal_param.t_fine = x1 + x2;
  temperature = ( p_bme280->cal_param.t_fine * 5 + 128) >> 8;

  return temperature;
}


u32 bme280_compensate_pressure_int32( struct bme280_t *p_bme280, s32 uncomp_pressure)
{
  s32 x1       = BME280_INIT_VALUE;
  s32 x2       = BME280_INIT_VALUE;
  u32 pressure = BME280_INIT_VALUE;

  s32 t_fine  = (s32)p_bme280->cal_param.t_fine;
  u16 dig_P1  = p_bme280->cal_param.dig_P1;
  s16 dig_P2  = p_bme280->cal_param.dig_P2;
  s16 dig_P3  = p_bme280->cal_param.dig_P3;
  s16 dig_P4  = p_bme280->cal_param.dig_P4;
  s16 dig_P5  = p_bme280->cal_param.dig_P5;
  s16 dig_P6  = p_bme280->cal_param.dig_P6;
  s16 dig_P7  = p_bme280->cal_param.dig_P7;
  s16 dig_P8  = p_bme280->cal_param.dig_P8;
  s16 dig_P9  = p_bme280->cal_param.dig_P9;

  x1 = (t_fine >> 1 ) - (s32)64000;
  x2 = ((( x1 >> 2 ) * ( x1 >> 2 )) >> 11 ) * ((s32)dig_P6);
  x2 = x2 + ((x1 * ((s32)dig_P5)) << 1 );
  x2 = (x2 >> 2 ) + (((s32)dig_P4)<< 16);
  x1 = (((dig_P3 * (((x1 >> 2) * (x1 >> 2)) >> 13)) >> 3) + ((((s32)dig_P2) * x1) >> 1 )) >> 18;
  x1 = ((((32768 + x1)) * ((s32)dig_P1)) >> 15);

  pressure = (((u32)(((s32)1048576) - uncomp_pressure) - (x2 >> 12 ))) * 3125;

  if (pressure < 0x80000000){ /* Avoid exception caused by division by zero */
    if (x1 != 0) {
      pressure = ( pressure << 1 ) / ((u32) x1);
    } else {
      return BME280_INVALID_DATA;
    }
  } else {    /* Avoid exception caused by division by zero */
    if (x1 != 0 ) {
      pressure = (pressure / (u32) x1) * 2;
    } else { 
      return BME280_INVALID_DATA;
    }
  }

  x1 = (((s32)dig_P9) * ((s32)(((pressure >> 3 ) * (pressure >> 3 )) >> 13 ))) >> 12;
  x2 = (((s32)(pressure >> 2 )) * ((s32)dig_P8)) >> 13;
  pressure = (u32)((s32)pressure + ((x1 + x2 + dig_P7) >> 4));

  return pressure;
}


u32 bme280_compensate_humidity_int32( struct bme280_t *p_bme280, s32 uncomp_humidity)
{
  s32 x1 = BME280_INIT_VALUE;

  u8  dig_H1 = p_bme280->cal_param.dig_H1;
  s16 dig_H2 = p_bme280->cal_param.dig_H2;
  u8  dig_H3 = p_bme280->cal_param.dig_H3;
  s16 dig_H4 = p_bme280->cal_param.dig_H4;
  s16 dig_H5 = p_bme280->cal_param.dig_H5;
  s8  dig_H6 = p_bme280->cal_param.dig_H6;

  x1 = (p_bme280->cal_param.t_fine - ((s32)76800));
  x1 = (((((uncomp_humidity << 14) - (((s32)dig_H4) << 20 ) - (((s32)dig_H5) * x1)) + ((s32)16384)) >> 15 )
        * (((((((x1 * ((s32)dig_H6)) >> 10) * (((x1* ((s32)dig_H3)) >> 11 ) + ((s32)32768)))
              >> 10) + ((s32)2097152)) * ((s32)dig_H2) + 8192) >> 14));

  x1 = (x1 - (((((x1 >> 15) * (x1 >> 15)) >> 7) * ((s32)dig_H1)) >> 4));
  x1 = (x1 < 0 ? 0 : x1 );
  x1 = (x1 > 419430400 ? 419430400 : x1);
  return (u32)(x1 >> 12);
}

s8 bme280_set_standby_durn( struct bme280_t *p_bme280, u8 num )
{

  s8 status   = ERROR;
  u8 data     = 0;
  u8 dev_addr = p_bme280->dev_addr;

  BME280_WR_FUNC = p_bme280->bus_write;

  status  = bme280_get_reg_value( p_bme280, &data, BME280_CONFIG_REG, 0xF, 0 );
  data    = BME280_SET_BITSLICE(data, BME280_CONFIG_REG_TSB, num);
  status += write( dev_addr, BME280_CONFIG_REG_TSB__REG, &data, 1);

  return status;
}


s8 bme280_get_standby_durn(struct bme280_t *p_bme280, u8 *standby_durn)
{
  
  s8 status = ERROR;
  u8 data   = 0;

  u8 dev_addr    = p_bme280->dev_addr;
  BME280_RD_FUNC = p_bme280->bus_read;

  status        = read ( dev_addr, BME280_CONFIG_REG_TSB__REG, &data, 1);
  *standby_durn = BME280_GET_BITSLICE( data, BME280_CONFIG_REG_TSB);
  return status;
}


s8 bme280_initialize(struct bme280_t *p_bme280)
{
  s8 status = ERROR;

  status = bme280_get_calib_param( p_bme280 );

  return status;
}

