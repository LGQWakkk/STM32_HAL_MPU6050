#pragma once

#include "main.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B	
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	    //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	    //IIC地址寄存器(默认数值0x68，只读)
// #define	MPU6050_ADDR	0x68	//IIC写入时的地址字节数据，+1为读取 原始地址
#define	MPU6050_ADDR	  0xD0	  //IIC写入时的地址字节数据，+1为读取 原始地址
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A
#define INT_PIN_CFG     0x37
#define FIFO_COUNT_H    0x72
#define FIFO_COUNT_L    0x73
#define FIFO_R_W        0x74
#define USER_CTRL       0x6A
#define FIFO_EN         0x23

#define INT_FF_EN               1<<7
#define INT_MOT_EN              1<<6
#define INT_ZMOT_EN             1<<5
#define INT_FIFO_OFLOW_EN       1<<4
#define INT_I2C_MST_INT_EN      1<<3
#define INT_DATA_RDY_EN         1<<0

#define GYRO_FS_250 0x00
#define GYRO_FS_500 0x08
#define GYRO_FS_1000 0x10
#define GYRO_FS_2000 0x18

#define ACC_FS_2 0x00
#define ACC_FS_4 0x08
#define ACC_FS_8 0x10
#define ACC_FS_16 0x18

///////////////////////////////////////register list//////////////////////////////////////////
#define MPU6050_AUX_VDDIO    0x01
#define MPU6050_SMPLRT_DIV   0x19

#define MPU6050_CONFIG 0x1A
typedef struct 
{
  uint8_t res :2;
  uint8_t ext_sync_set :3;
  uint8_t dlpf_cfg :3;
}mpu6050_config_t;

#define MPU6050_GYRO_CONFIG 0x1B
typedef struct 
{
  uint8_t xg_st :1;
  uint8_t yg_st :1;
  uint8_t zg_st :1;
  uint8_t fs_sel :2;
  uint8_t res :3;
}mpu6050_gyro_config_t;

#define MPU6050_ACCEL_CONFIG 0x1C
typedef struct 
{
  uint8_t xa_st :1;
  uint8_t ya_st :1;
  uint8_t za_st :1;
  uint8_t afs_sel :2;
  uint8_t accel_hpf :3;
}mpu6050_accel_config_t;

#define MPU6050_FF_THR          0x1d
#define MPU6050_FF_DUR          0x1e
#define MPU6050_MOT_THR         0x1f
#define MPU6050_MOT_DUR         0x20
#define MPU6050_ZRMOT_THR       0x21
#define MPU6050_ZRMOT_DUR       0X22

#define MPU6050_FIFO_EN 0x23
typedef struct 
{
  uint8_t temp_fifo_en :1;
  uint8_t xg_fifo_en :1;
  uint8_t yg_fifo_en :1;
  uint8_t zg_fifo_en :1;
  uint8_t accel_fifo_en :1;
  uint8_t slv2_fifo_en :1;
  uint8_t slv1_fifo_en :1;
  uint8_t slv0_fifo_en :1;
}mpu6050_fifo_en_t;

#define MPU6050_I2C_MST_CTRL        0X24
#define MPU6050_I2C_SLV0_ADDR       0X25
#define MPU6050_I2C_SLV0_REG        0X26
#define MPU6050_I2C_SLV0_CTRL       0X27

#define MPU6050_I2C_SLV1_ADDR       0X28
#define MPU6050_I2C_SLV1_REG        0X29
#define MPU6050_I2C_SLV1_CTRL       0X2A

#define MPU6050_I2C_SLV2_ADDR       0X2B
#define MPU6050_I2C_SLV2_REG        0X2C
#define MPU6050_I2C_SLV2_CTRL       0X2D

#define MPU6050_I2C_SLV3_ADDR       0X2E
#define MPU6050_I2C_SLV3_REG        0X2F
#define MPU6050_I2C_SLV3_CTRL       0X30

#define MPU6050_I2C_SLV4_ADDR       0X31
#define MPU6050_I2C_SLV4_REG        0X32
#define MPU6050_I2C_SLV4_DO         0X33
#define MPU6050_I2C_SLV4_CTRL       0X34
#define MPU6050_I2C_SLV4_DI         0X35

#define MPU6050_I2C_MST_STATUS      0X36

#define MPU6050_INT_PIN_CFG 0x37
typedef struct 
{
  uint8_t int_level :1;
  uint8_t int_open :1;
  uint8_t latch_int_en :1;
  uint8_t int_rd_clear :1;
  uint8_t fsync_int_level :1;
  uint8_t fsync_int_en :1;
  uint8_t i2c_bypass_en :1;
  uint8_t clkout_en :1;
}mpu6050_int_pin_cfg_t;

#define MPU6050_INT_ENABLE 0x38
typedef struct 
{
  uint8_t ff_en :1;
  uint8_t mot_en :1;
  uint8_t zmot_en :1;
  uint8_t fifo_oflow_en :1;
  uint8_t i2c_mst_int_en :1;
  uint8_t res :2;
  uint8_t data_rdy_en :1;
}mpu6050_int_enable_t;

#define MPU6050_INT_STATUS 0x3A
typedef struct 
{
  uint8_t ff_int :1;
  uint8_t mot_int :1;
  uint8_t zmot_int :1;
  uint8_t fifo_oflow_int :1;
  uint8_t i2c_mst_int :1;
  uint8_t res :2;
  uint8_t data_rdy_int :1;
}mpu6050_int_status_t;

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_SIGNAL_PATH_RESET 0x68
typedef struct 
{
  uint8_t res :5;
  uint8_t gyro_reset :1;
  uint8_t accel_reset :1;
  uint8_t temp_reset :1;
}mpu6050_signal_path_reset_t;

#define MPU6050_USER_CTRL 0x6A
typedef struct 
{
  uint8_t res_1 :1;
  uint8_t fifo_en :1;
  uint8_t i2c_mst_en :1;
  uint8_t i2c_if_dis :1;
  uint8_t res_2 :1;
  uint8_t fifo_reset :1;
  uint8_t i2c_mst_reset :1;
  uint8_t sig_cond_reset :1;
}mpu6050_user_ctrl_t;

#define MPU6050_PWR_MGMT_1 0x6B
typedef struct
{
    uint8_t device_reset      : 1;
    uint8_t sleep             : 1;
    uint8_t cycle             : 1;
    uint8_t res               : 1;
    uint8_t temp_dis          : 1;
    uint8_t clock_sel         : 3;
}mpu6050_pwr_mgmt_1_t;

#define MPU6050_PWR_MGMT_2 0x6C
typedef struct
{
    uint8_t lp_wake_ctrl      : 2;
    uint8_t stby_xa           : 1;
    uint8_t stby_ya           : 1;
    uint8_t stby_za           : 1;
    uint8_t stby_xg           : 1;
    uint8_t stby_yg           : 1;
    uint8_t stby_zg           : 1;
}mpu6050_pwr_mgmt_2_t;

#define MPU6050_FIFO_COUNT_H 0x72
#define MPU6050_FIFO_COUNT_L 0x73
#define MPU6050_FIFO_R_W  0x74
#define MPU6050_WHO_AM_I 0x75
//////////////////////////////////////////////////////////////////////////////////////////////////
//I2C function
uint8_t i2c_probe(void);
uint8_t mpu6050_is_ready(uint8_t retry_count);
HAL_StatusTypeDef i2c_write_reg_byte(uint8_t reg, uint8_t data);
HAL_StatusTypeDef i2c_read_reg_byte(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef i2c_read_reg_bytes(uint8_t start_addr, uint8_t *output_buffer, uint8_t len);
///////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t mpu6050_read_id(void);
void mpu6050_init();
//原始数据读取
void mpu6050_read_all(uint8_t *acc, uint8_t *tmp, uint8_t *gyro);
void mpu6050_read_acc(int16_t *acc);
void mpu6050_read_tmp(int16_t *tmp);
void mpu6050_read_gyro(int16_t *gyro);
void mpu6050_read_only_acc(int16_t *acc);
void mpu6050_read_only_tmp(int16_t *tmp);
void mpu6050_read_only_gyro(int16_t *gyro);
//读取mg单位加速度
float mpu6050_from_fs2g_to_mg(int16_t lsb);
float mpu6050_from_fs4g_to_mg(int16_t lsb);
float mpu6050_from_fs8g_to_mg(int16_t lsb);
float mpu6050_from_fs16g_to_mg(int16_t lsb);
//读取g单位加速度
float mpu6050_from_fs2g_to_g(int16_t lsb);
float mpu6050_from_fs4g_to_g(int16_t lsb);
float mpu6050_from_fs8g_to_g(int16_t lsb);
float mpu6050_from_fs16g_to_g(int16_t lsb);
//读取mdps单位角速度
float mpu6050_from_fs250dps_to_mdps(int16_t lsb);
float mpu6050_from_fs500dps_to_mdps(int16_t lsb);
float mpu6050_from_fs1000dps_to_mdps(int16_t lsb);
float mpu6050_from_fs2000dps_to_mdps(int16_t lsb);
//读取dps单位角速度
float mpu6050_from_fs250dps_to_dps(int16_t lsb);
float mpu6050_from_fs500dps_to_dps(int16_t lsb);
float mpu6050_from_fs1000dps_to_dps(int16_t lsb);
float mpu6050_from_fs2000dps_to_dps(int16_t lsb);

float mpu6050_from_lsb_to_celsius(int16_t lsb);

void mpu6050_int_en(uint8_t int_type);
uint8_t mpu6050_read_status(void);
uint8_t mpu6050_is_data_ready_by_reg(void);
uint8_t mpu6050_is_data_ready_by_gpio(void);
uint16_t mpu6050_get_fifo_count(void);
uint16_t mpu6050_read_fifo(uint8_t *output);
