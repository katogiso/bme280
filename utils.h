#ifndef __BME280_H__
#define __BME280_H__

//#include <linux/types.h>
//#include <linux/math64.h>
#include <stdint.h>

typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;


#define BME280_ENABLE_FLOAT
#define BME280_ENABLE_INT64

/***************************************************************/
/**\name COMMON USED CONSTANTS      */
/***************************************************************/
/* Constants */
#define BME280_NULL                            (0)
/* shift definitions*/
#define BME280_SHIFT_BIT_POSITION_BY_01_BIT    (1)
#define BME280_SHIFT_BIT_POSITION_BY_02_BITS   (2)
#define BME280_SHIFT_BIT_POSITION_BY_03_BITS   (3)
#define BME280_SHIFT_BIT_POSITION_BY_04_BITS   (4)
#define BME280_SHIFT_BIT_POSITION_BY_07_BITS   (7)
#define BME280_SHIFT_BIT_POSITION_BY_08_BITS   (8)
#define BME280_SHIFT_BIT_POSITION_BY_10_BITS   (10)
#define BME280_SHIFT_BIT_POSITION_BY_11_BITS   (11)
#define BME280_SHIFT_BIT_POSITION_BY_12_BITS   (12)
#define BME280_SHIFT_BIT_POSITION_BY_13_BITS   (13)
#define BME280_SHIFT_BIT_POSITION_BY_14_BITS   (14)
#define BME280_SHIFT_BIT_POSITION_BY_15_BITS   (15)
#define BME280_SHIFT_BIT_POSITION_BY_16_BITS   (16)
#define BME280_SHIFT_BIT_POSITION_BY_17_BITS   (17)
#define BME280_SHIFT_BIT_POSITION_BY_18_BITS   (18)
#define BME280_SHIFT_BIT_POSITION_BY_19_BITS   (19)
#define BME280_SHIFT_BIT_POSITION_BY_20_BITS   (20)
#define BME280_SHIFT_BIT_POSITION_BY_25_BITS   (25)
#define BME280_SHIFT_BIT_POSITION_BY_31_BITS   (31)
#define BME280_SHIFT_BIT_POSITION_BY_33_BITS   (33)
#define BME280_SHIFT_BIT_POSITION_BY_35_BITS   (35)
#define BME280_SHIFT_BIT_POSITION_BY_47_BITS   (47)

/* numeric definitions */
#define BME280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH (26)
#define BME280_HUMIDITY_CALIB_DATA_LENGTH             (7)
#define BME280_GEN_READ_WRITE_DATA_LENGTH             (1)
#define BME280_HUMIDITY_DATA_LENGTH                   (2)
#define BME280_TEMPERATURE_DATA_LENGTH                (3)
#define BME280_PRESSURE_DATA_LENGTH                   (3)
#define BME280_ALL_DATA_FRAME_LENGTH                  (8)
#define BME280_INIT_VALUE                             (0)
#define BME280_CHIP_ID_READ_COUNT                     (5)
#define BME280_INVALID_DATA                           (0)

/****************************************************/
/**\name    ERROR CODE DEFINITIONS  */
/***************************************************/
#define SUCCESS                  ((u8)0)
#define ERROR                    ((s8)-1)

/****************************************************/
/**\name    CHIP ID DEFINITIONS  */
/***************************************************/
#define BME280_CHIP_ID           (0x60)

/****************************************************/
/**\name    I2C ADDRESS DEFINITIONS  */
/***************************************************/
#define BME280_I2C_ADDRESS1      (0x76)
#define BME280_I2C_ADDRESS2      (0x77)
/****************************************************/
/**\name    POWER MODE DEFINITIONS  */
/***************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE        (0x00)
#define BME280_FORCED_MODE       (0x01)
#define BME280_NORMAL_MODE       (0x03)
#define BME280_SOFT_RESET_CODE   (0xB6)
/****************************************************/
/**\name    STANDBY DEFINITIONS  */
/***************************************************/
#define BME280_STANDBY_TIME_1_MS    (0x00)
#define BME280_STANDBY_TIME_63_MS   (0x01)
#define BME280_STANDBY_TIME_125_MS  (0x02)
#define BME280_STANDBY_TIME_250_MS  (0x03)
#define BME280_STANDBY_TIME_500_MS  (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS   (0x06)
#define BME280_STANDBY_TIME_20_MS   (0x07)
/****************************************************/
/**\name    OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME280_OVERSAMP_SKIPPED     (0x00)
#define BME280_OVERSAMP_1X          (0x01)
#define BME280_OVERSAMP_2X          (0x02)
#define BME280_OVERSAMP_4X          (0x03)
#define BME280_OVERSAMP_8X          (0x04)
#define BME280_OVERSAMP_16X         (0x05)

/****************************************************/
/**\name    FILTER DEFINITIONS  */
/***************************************************/
#define BME280_FILTER_COEFF_OFF  (0x00)
#define BME280_FILTER_COEFF_2    (0x01)
#define BME280_FILTER_COEFF_4    (0x02)
#define BME280_FILTER_COEFF_8    (0x03)
#define BME280_FILTER_COEFF_16   (0x04)

/****************************************************/
/**\name    DELAY DEFINITIONS  */
/***************************************************/
#define T_INIT_MAX              (20) /* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX  (37) /* 37/16 = 2.3125 ms*/
#define T_SETUP_PRESSURE_MAX    (10) /* 10/16 = 0.625 ms */
#define T_SETUP_HUMIDITY_MAX    (10) /* 10/16 = 0.625 ms */

/****************************************************/
/**\name    DEFINITIONS FOR ARRAY SIZE OF DATA   */
/***************************************************/
#define BME280_HUMIDITY_DATA_SIZE         (2)
#define BME280_TEMPERATURE_DATA_SIZE      (3)
#define BME280_PRESSURE_DATA_SIZE         (3)
#define BME280_DATA_FRAME_SIZE            (8)

#define BME280_CALIB_DATA_SIZE            (26)

#define BME280_TEMPERATURE_MSB_DATA       (0)
#define BME280_TEMPERATURE_LSB_DATA       (1)
#define BME280_TEMPERATURE_XLSB_DATA      (2)
#define BME280_PRESSURE_MSB_DATA          (0)
#define BME280_PRESSURE_LSB_DATA          (1)
#define BME280_PRESSURE_XLSB_DATA         (2)
#define BME280_HUMIDITY_MSB_DATA          (0)
#define BME280_HUMIDITY_LSB_DATA          (1)

#define BME280_DATA_FRAME_PRESSURE_MSB_BYTE      (0)
#define BME280_DATA_FRAME_PRESSURE_LSB_BYTE      (1)
#define BME280_DATA_FRAME_PRESSURE_XLSB_BYTE     (2)
#define BME280_DATA_FRAME_TEMPERATURE_MSB_BYTE   (3)
#define BME280_DATA_FRAME_TEMPERATURE_LSB_BYTE   (4)
#define BME280_DATA_FRAME_TEMPERATURE_XLSB_BYTE  (5)
#define BME280_DATA_FRAME_HUMIDITY_MSB_BYTE      (6)
#define BME280_DATA_FRAME_HUMIDITY_LSB_BYTE      (7)

/****************************************************/
/**\name    ARRAY PARAMETER FOR CALIBRATION     */
/***************************************************/
#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB (0)
#define BME280_TEMPERATURE_CALIB_DIG_T1_MSB (1)
#define BME280_TEMPERATURE_CALIB_DIG_T2_LSB (2)
#define BME280_TEMPERATURE_CALIB_DIG_T2_MSB (3)
#define BME280_TEMPERATURE_CALIB_DIG_T3_LSB (4)
#define BME280_TEMPERATURE_CALIB_DIG_T3_MSB (5)
#define BME280_PRESSURE_CALIB_DIG_P1_LSB    (6)
#define BME280_PRESSURE_CALIB_DIG_P1_MSB    (7)
#define BME280_PRESSURE_CALIB_DIG_P2_LSB    (8)
#define BME280_PRESSURE_CALIB_DIG_P2_MSB    (9)
#define BME280_PRESSURE_CALIB_DIG_P3_LSB    (10)
#define BME280_PRESSURE_CALIB_DIG_P3_MSB    (11)
#define BME280_PRESSURE_CALIB_DIG_P4_LSB    (12)
#define BME280_PRESSURE_CALIB_DIG_P4_MSB    (13)
#define BME280_PRESSURE_CALIB_DIG_P5_LSB    (14)
#define BME280_PRESSURE_CALIB_DIG_P5_MSB    (15)
#define BME280_PRESSURE_CALIB_DIG_P6_LSB    (16)
#define BME280_PRESSURE_CALIB_DIG_P6_MSB    (17)
#define BME280_PRESSURE_CALIB_DIG_P7_LSB    (18)
#define BME280_PRESSURE_CALIB_DIG_P7_MSB    (19)
#define BME280_PRESSURE_CALIB_DIG_P8_LSB    (20)
#define BME280_PRESSURE_CALIB_DIG_P8_MSB    (21)
#define BME280_PRESSURE_CALIB_DIG_P9_LSB    (22)
#define BME280_PRESSURE_CALIB_DIG_P9_MSB    (23)
#define BME280_HUMIDITY_CALIB_DIG_H1        (25)
#define BME280_HUMIDITY_CALIB_DIG_H2_LSB    (0)
#define BME280_HUMIDITY_CALIB_DIG_H2_MSB    (1)
#define BME280_HUMIDITY_CALIB_DIG_H3        (2)
#define BME280_HUMIDITY_CALIB_DIG_H4_MSB    (3)
#define BME280_HUMIDITY_CALIB_DIG_H4_LSB    (4)
#define BME280_HUMIDITY_CALIB_DIG_H5_MSB    (5)
#define BME280_HUMIDITY_CALIB_DIG_H6        (6)
#define BME280_MASK_DIG_H4                  (0x0F)

/****************************************************/
/**\name    CALIBRATION REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG  (0x88)
#define BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG  (0x89)
#define BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG  (0x8A)
#define BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG  (0x8B)
#define BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG  (0x8C)
#define BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG  (0x8D)
#define BME280_PRESSURE_CALIB_DIG_P1_LSB_REG     (0x8E)
#define BME280_PRESSURE_CALIB_DIG_P1_MSB_REG     (0x8F)
#define BME280_PRESSURE_CALIB_DIG_P2_LSB_REG     (0x90)
#define BME280_PRESSURE_CALIB_DIG_P2_MSB_REG     (0x91)
#define BME280_PRESSURE_CALIB_DIG_P3_LSB_REG     (0x92)
#define BME280_PRESSURE_CALIB_DIG_P3_MSB_REG     (0x93)
#define BME280_PRESSURE_CALIB_DIG_P4_LSB_REG     (0x94)
#define BME280_PRESSURE_CALIB_DIG_P4_MSB_REG     (0x95)
#define BME280_PRESSURE_CALIB_DIG_P5_LSB_REG     (0x96)
#define BME280_PRESSURE_CALIB_DIG_P5_MSB_REG     (0x97)
#define BME280_PRESSURE_CALIB_DIG_P6_LSB_REG     (0x98)
#define BME280_PRESSURE_CALIB_DIG_P6_MSB_REG     (0x99)
#define BME280_PRESSURE_CALIB_DIG_P7_LSB_REG     (0x9A)
#define BME280_PRESSURE_CALIB_DIG_P7_MSB_REG     (0x9B)
#define BME280_PRESSURE_CALIB_DIG_P8_LSB_REG     (0x9C)
#define BME280_PRESSURE_CALIB_DIG_P8_MSB_REG     (0x9D)
#define BME280_PRESSURE_CALIB_DIG_P9_LSB_REG     (0x9E)
#define BME280_PRESSURE_CALIB_DIG_P9_MSB_REG     (0x9F)

#define BME280_HUMIDITY_CALIB_DIG_H1_REG         (0xA1)

#define BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG     (0xE1)
#define BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG     (0xE2)
#define BME280_HUMIDITY_CALIB_DIG_H3_REG         (0xE3)
#define BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG     (0xE4)
#define BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG     (0xE5)
#define BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG     (0xE6)
#define BME280_HUMIDITY_CALIB_DIG_H6_REG         (0xE7)
/****************************************************/
/**\name    REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
#define BME280_CHIP_ID_REG                   (0xD0)
#define BME280_RST_REG                       (0xE0)
#define BME280_STAT_REG                      (0xF3)
#define BME280_CTRL_MEAS_REG                 (0xF4)
#define BME280_CTRL_HUMIDITY_REG             (0xF2)
#define BME280_CONFIG_REG                    (0xF5)
#define BME280_PRESSURE_MSB_REG              (0xF7)
#define BME280_PRESSURE_LSB_REG              (0xF8)
#define BME280_PRESSURE_XLSB_REG             (0xF9)
#define BME280_TEMPERATURE_MSB_REG           (0xFA)
#define BME280_TEMPERATURE_LSB_REG           (0xFB)
#define BME280_TEMPERATURE_XLSB_REG          (0xFC)
#define BME280_HUMIDITY_MSB_REG              (0xFD)
#define BME280_HUMIDITY_LSB_REG              (0xFE)


#define BME280_GET_BITSLICE(regvar, bitname) ((regvar & bitname##__MSK) >> bitname##__POS)
#define BME280_SET_BITSLICE(regvar, bitname, val) (regvar & ~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK)

/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS  */
/***************************************************/
/* Status Register */
#define BME280_STAT_REG_MEASURING__POS  (3)
#define BME280_STAT_REG_MEASURING__MSK  (0x08)
#define BME280_STAT_REG_MEASURING__LEN  (1)
#define BME280_STAT_REG_MEASURING__REG  (BME280_STAT_REG)

#define BME280_STAT_REG_IM_UPDATE__POS  (0)
#define BME280_STAT_REG_IM_UPDATE__MSK  (0x01)
#define BME280_STAT_REG_IM_UPDATE__LEN  (1)
#define BME280_STAT_REG_IM_UPDATE__REG  (BME280_STAT_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR TEMPERATURE OVERSAMPLING  */
/***************************************************/
/* Control Measurement Register */
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS (5)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK (0xE0)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG (BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR PRESSURE OVERSAMPLING  */
/***************************************************/
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS (2)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK (0x1C)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG (BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR POWER MODE  */
/***************************************************/
#define BME280_CTRL_MEAS_REG_POWER_MODE__POS  (0)
#define BME280_CTRL_MEAS_REG_POWER_MODE__MSK  (0x03)
#define BME280_CTRL_MEAS_REG_POWER_MODE__LEN  (2)
#define BME280_CTRL_MEAS_REG_POWER_MODE__REG  (BME280_CTRL_MEAS_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR HUMIDITY OVERSAMPLING  */
/***************************************************/
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__POS (0)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__MSK (0x07)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__LEN (3)
#define BME280_CTRL_HUMIDITY_REG_OVERSAMP_HUMIDITY__REG (BME280_CTRL_HUMIDITY_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR STANDBY TIME  */
/***************************************************/
/* Configuration Register */
#define BME280_CONFIG_REG_TSB__POS (5)
#define BME280_CONFIG_REG_TSB__MSK (0xE0)
#define BME280_CONFIG_REG_TSB__LEN (3)
#define BME280_CONFIG_REG_TSB__REG (BME280_CONFIG_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR FILTER */
/***************************************************/
#define BME280_CONFIG_REG_FILTER__POS (2)
#define BME280_CONFIG_REG_FILTER__MSK (0x1C)
#define BME280_CONFIG_REG_FILTER__LEN (3)
#define BME280_CONFIG_REG_FILTER__REG (BME280_CONFIG_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR SPI ENABLE  */
/***************************************************/
#define BME280_CONFIG_REG_SPI3_ENABLE__POS (0)
#define BME280_CONFIG_REG_SPI3_ENABLE__MSK (0x01)
#define BME280_CONFIG_REG_SPI3_ENABLE__LEN (1)
#define BME280_CONFIG_REG_SPI3_ENABLE__REG (BME280_CONFIG_REG)
/****************************************************/
/**\name    BIT MASK, LENGTH AND POSITION DEFINITIONS
   FOR PRESSURE AND TEMPERATURE DATA  */
/***************************************************/
/* Data Register */
#define BME280_PRESSURE_XLSB_REG_DATA__POS (4)
#define BME280_PRESSURE_XLSB_REG_DATA__MSK (0xF0)
#define BME280_PRESSURE_XLSB_REG_DATA__LEN (4)
#define BME280_PRESSURE_XLSB_REG_DATA__REG (BME280_PRESSURE_XLSB_REG)

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK (0xF0)
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__REG (BME280_TEMPERATURE_XLSB_REG)


/****************************************************/
/**\name BUS READ AND WRITE FUNCTION POINTERS */
/***************************************************/
#define BME280_WR_FUNC_PTR s8 (*bus_write)(u8, u8, u8 *, u8)
#define BME280_RD_FUNC_PTR s8 (*bus_read)(u8, u8, u8 *, u8)

#define BME280_WR_FUNC s8 (*write)(u8, u8, u8 *, u8)
#define BME280_RD_FUNC s8 (*read)(u8, u8, u8 *, u8)

#define BME280_MDELAY_DATA_TYPE    int
#define BME280_3MS_DELAY        (3)
#define BME280_REGISTER_READ_DELAY (1)


/**************************************************************/
/**\name STRUCTURE DEFINITIONS                         */
/**************************************************************/
struct bme280_calibration_param_t 
{
  u16 dig_T1;
  s16 dig_T2;
  s16 dig_T3;
  u16 dig_P1;
  s16 dig_P2;
  s16 dig_P3;
  s16 dig_P4;
  s16 dig_P5;
  s16 dig_P6;
  s16 dig_P7;
  s16 dig_P8;
  s16 dig_P9;
  
  u8  dig_H1;
  s16 dig_H2;
  u8  dig_H3;
  s16 dig_H4;
  s16 dig_H5;
  s8  dig_H6;
  
  s32 t_fine;
};

struct bme280_t {
  struct bme280_calibration_param_t cal_param;
  
  u8 chip_id;
  u8 dev_addr;
  
  u8 oversamp_temperature;
  u8 oversamp_pressure;
  u8 oversamp_humidity;
  u8 ctrl_hum_reg;
  u8 ctrl_meas_reg;
  u8 config_reg;
  
  BME280_WR_FUNC_PTR;
  BME280_RD_FUNC_PTR;
  void (*delay_msec)(BME280_MDELAY_DATA_TYPE);
};

#define BME280_CAST_TO_U16(array, upper_pos, lower_pos)  (u16)((((u16)((u8)array[upper_pos])) << 8) | array[lower_pos])
#define BME280_CAST_TO_S16(array, upper_pos, lower_pos)  (s16)((((s16)((s8)array[upper_pos])) << 8) | array[lower_pos])

s8  bme280_get_reg_value( struct bme280_t *p_bme280, u8 *value, u8 addr, u8 msk, u8 pos );
s8  bme280_initialize(struct bme280_t *p_bme280);
s8  bme280_id_read(struct bme280_t *p_bme280);
s8  bme280_get_calib_param(struct bme280_t *p_bme280);
s8  bme280_get_power_mode( struct bme280_t *p_bme280, u8 *power_mode );
s8  bme280_set_power_mode( struct bme280_t *p_bme280, u8 power_mode );
s8  bme280_set_oversamp_temperature( struct bme280_t *p_bme280, u8 mode );
s8  bme280_set_oversamp_pressure( struct bme280_t *p_bme280, u8 mode );
s8  bme280_set_oversamp_humidity( struct bme280_t *p_bme280, u8 mode );
s8  bme280_set_standby_durn( struct bme280_t *p_bme280, u8 num );
s8  bme280_read_uncomp_pressure( struct bme280_t *p_bme280, s32 *uncomp_pressure );
s8  bme280_read_uncomp_temperature( struct bme280_t *p_bme280, s32 *uncomp_temperature);
s8  bme280_read_uncomp_humidity( struct bme280_t *p_bme280, s32 *uncomp_humidity);
s32 bme280_compensate_temperature_int32( struct bme280_t *p_bme280, s32 uncomp_temperature);
u32 bme280_compensate_pressure_int32( struct bme280_t *p_bme280, s32 uncomp_pressure);
u32 bme280_compensate_humidity_int32( struct bme280_t *p_bme280, s32 uncomp_humidity);
s8  bme280_get_standby_durn(struct bme280_t *p_bme280, u8 *standby_durn);

#endif
