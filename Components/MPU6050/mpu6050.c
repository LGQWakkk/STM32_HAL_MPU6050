#include "mpu6050.h"
#include "i2c.h"

// Your IIC handle
#define HAL_I2C_HANDLE hi2c2

// Original Data
int16_t raw_acc[3];
int16_t raw_temp;
int16_t raw_gyro[3];

///////////////////////////////////////Basic I2C Functions////////////////////////////////////////
uint8_t i2c_probe(void)
{
  uint8_t addr=0xff;
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&HAL_I2C_HANDLE, i, 1, 1000) == HAL_OK)
		{
			addr = i;
			return i;
		}
	}
	return 0xff;
}

//对MPU6050是否连接正常进行检查
uint8_t mpu6050_is_ready(uint8_t retry_count)
{
  uint8_t ret=0;
  ret = (uint8_t)HAL_I2C_IsDeviceReady(&HAL_I2C_HANDLE, MPU6050_ADDR, retry_count, 100);
  return ret;
}

HAL_StatusTypeDef i2c_write_reg_byte(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&HAL_I2C_HANDLE, MPU6050_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (&data), 1, 100);
  if (status != HAL_OK) {
    printf("I2C Write Error! code: %d\r\n", status);
    // HAL_OK       = 0x00U,
    // HAL_ERROR    = 0x01U,
    // HAL_BUSY     = 0x02U,
    // HAL_TIMEOUT  = 0x03U
  }
  while (HAL_I2C_GetState(&HAL_I2C_HANDLE) != HAL_I2C_STATE_READY);

  // /* Check if the deice is ready for a new operation */
  // while (HAL_I2C_IsDeviceReady(&I2C_Handle, EEPROM_ADDRESS,
  //     EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT);
  // /* Wait for the end of the transfer */
  // while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY);
  return status;
}

HAL_StatusTypeDef i2c_read_reg_byte(uint8_t reg, uint8_t *data)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&HAL_I2C_HANDLE, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
  if (status != HAL_OK) {
    printf("I2C Read Error! code: %d\r\n", status);
    // HAL_OK       = 0x00U,
    // HAL_ERROR    = 0x01U,
    // HAL_BUSY     = 0x02U,
    // HAL_TIMEOUT  = 0x03U
  }
  return status;
}

HAL_StatusTypeDef i2c_read_reg_bytes(uint8_t start_addr, uint8_t *output_buffer, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&HAL_I2C_HANDLE, MPU6050_ADDR, start_addr, I2C_MEMADD_SIZE_8BIT, output_buffer, len, 100);
  if (status != HAL_OK) {
    printf("I2C Read Bytes Error! code: %d\r\n", status);
    // HAL_OK       = 0x00U,
    // HAL_ERROR    = 0x01U,
    // HAL_BUSY     = 0x02U,
    // HAL_TIMEOUT  = 0x03U
  }
  return status;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t mpu6050_read_id(void)
{
    uint8_t id = 0;
    i2c_read_reg_byte(WHO_AM_I, &id);
    return id;
}

void mpu6050_init() 
{
  //config the interrupt pin
    //不使用中断方式:
    // gpio_config_t io_conf = {};
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pin_bit_mask = MPU6050_INT_PIN_SEL;
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);
    //使用中断方式:
    // gpio_config_t io_conf = {};
    // io_conf.intr_type = GPIO_INTR_POSEDGE;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pin_bit_mask = MPU6050_INT_PIN_SEL;
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);

    // 修改为你的默认值
    i2c_write_reg_byte(PWR_MGMT_1, 0x01);//use gyro clk
    i2c_write_reg_byte(SMPLRT_DIV, 0x09);//设置分频值(Rate = 1KHz) 0x07
    i2c_write_reg_byte(CONFIG, 0x06);//不使用DLPF
    i2c_write_reg_byte(GYRO_CONFIG, GYRO_FS_250);//250deg/s
    i2c_write_reg_byte(ACCEL_CONFIG, ACC_FS_2);//2g
}

// 获取fifo中缓存数据个数(bytes)
uint16_t mpu6050_get_fifo_count(void)
{
  uint8_t tmp[2];
  uint16_t fifo_count = 0;
  i2c_read_reg_bytes(FIFO_COUNT_H, tmp, 2);
  fifo_count = (int16_t)((tmp[0] << 8) | tmp[1]);
  return fifo_count;
}

// output: 数据输出buffer
// return: 实际读取的字节数
uint16_t mpu6050_read_fifo(uint8_t *output)
{
  uint16_t fifo_count = mpu6050_get_fifo_count();
  if(fifo_count>0){
    i2c_read_reg_bytes(FIFO_R_W, output, fifo_count);
    return fifo_count;
  }else{
    return 0;
  }
}

// 寄存器读取是否有数据更新
uint8_t mpu6050_is_data_ready_by_reg(void)
{
    return mpu6050_read_status() & 0x01;
}

// 读取状态寄存器
uint8_t mpu6050_read_status(void)
{
    uint8_t status = 0;
    i2c_read_reg_byte(INT_STATUS, &status);
    return status;
}

// 中断使能
void mpu6050_int_en(uint8_t int_type)
{
    i2c_write_reg_byte(INT_ENABLE, int_type);
}

// 连续读取所有输出数据
void mpu6050_read_all(uint8_t *acc, uint8_t *tmp, uint8_t *gyro)
{
    uint8_t output[14];
    i2c_read_reg_bytes(ACCEL_XOUT_H, output, 14);
    if(acc){
        for(uint8_t i=0;i<6;i++)
        {
            acc[i] = output[i];
        }
    }
    if(tmp){
        for(uint8_t i=0;i<2;i++)
        {
            tmp[i] = output[i+6];
        }
    }
    if(gyro){
        for(uint8_t i=0;i<6;i++)
        {
            gyro[i] = output[i+8];
        }
    }
}

// 单独读取ACC
void mpu6050_read_only_acc(int16_t *acc)
{
    uint8_t output[6];
    i2c_read_reg_bytes(ACCEL_XOUT_H, output, 6);
    acc[0] = (int16_t)((output[0] << 8) | output[1]);
    acc[1] = (int16_t)((output[2] << 8) | output[3]);
    acc[2] = (int16_t)((output[4] << 8) | output[5]);
}

// 单独读取温度
void mpu6050_read_only_tmp(int16_t *tmp)
{
    uint8_t output[2];
    i2c_read_reg_bytes(TEMP_OUT_H, output, 2);
    tmp[0] = (int16_t)((output[0] << 8) | output[1]);
}

// 单独读取角速度
void mpu6050_read_only_gyro(int16_t *gyro)
{
    uint8_t output[6];
    i2c_read_reg_bytes(GYRO_XOUT_H, output, 6);
    gyro[0] = (int16_t)((output[0] << 8) | output[1]);
    gyro[1] = (int16_t)((output[2] << 8) | output[3]);
    gyro[2] = (int16_t)((output[4] << 8) | output[5]);
}

// 全部读取 分离ACC
void mpu6050_read_acc(int16_t *acc)
{
    uint8_t output[6];
    mpu6050_read_all(output, NULL, NULL);
    acc[0] = (int16_t)((output[0] << 8) | output[1]);
    acc[1] = (int16_t)((output[2] << 8) | output[3]);
    acc[2] = (int16_t)((output[4] << 8) | output[5]);
}

// 全部读取 分离TMP
void mpu6050_read_tmp(int16_t *tmp)
{
    uint8_t output[2];
    mpu6050_read_all(NULL, output, NULL);
    tmp[0] = (int16_t)((output[0] << 8) | output[1]);
}

// 全部读取 分离GYRO
void mpu6050_read_gyro(int16_t *gyro)
{
    uint8_t output[6];
    mpu6050_read_all(NULL, NULL, output);
    gyro[0] = (int16_t)((output[0] << 8) | output[1]);
    gyro[1] = (int16_t)((output[2] << 8) | output[3]);
    gyro[2] = (int16_t)((output[4] << 8) | output[5]);
}

//mg
float mpu6050_from_fs2g_to_mg(int16_t lsb)
{
  return ((float)lsb/16384*1000);
}
float mpu6050_from_fs4g_to_mg(int16_t lsb)
{
  return ((float)lsb/8192*1000);
}
float mpu6050_from_fs8g_to_mg(int16_t lsb)
{
  return ((float)lsb/4096*1000);
}
float mpu6050_from_fs16g_to_mg(int16_t lsb)
{
  return ((float)lsb/2048*1000);
}
//g
float mpu6050_from_fs2g_to_g(int16_t lsb)
{
  return ((float)lsb/16384.0);
}
float mpu6050_from_fs4g_to_g(int16_t lsb)
{
  return ((float)lsb/8192.0);
}
float mpu6050_from_fs8g_to_g(int16_t lsb)
{
  return ((float)lsb/4096.0);
}
float mpu6050_from_fs16g_to_g(int16_t lsb)
{
  return ((float)lsb/2048.0);
}
//mdps
float mpu6050_from_fs250dps_to_mdps(int16_t lsb)
{
  return ((float)lsb/131.0*1000.0f);
}
float mpu6050_from_fs500dps_to_mdps(int16_t lsb)
{
  return ((float)lsb/65.5*1000.0f);
}
float mpu6050_from_fs1000dps_to_mdps(int16_t lsb)
{
  return ((float)lsb/32.8*1000.0f);
}
float mpu6050_from_fs2000dps_to_mdps(int16_t lsb)
{
  return ((float)lsb/16.4*1000.0f);
}
//dps
float mpu6050_from_fs250dps_to_dps(int16_t lsb)
{
  return ((float)lsb/131.0);
}
float mpu6050_from_fs500dps_to_dps(int16_t lsb)
{
  return ((float)lsb/65.5);
}
float mpu6050_from_fs1000dps_to_dps(int16_t lsb)
{
  return ((float)lsb/32.8);
}
float mpu6050_from_fs2000dps_to_dps(int16_t lsb)
{
  return ((float)lsb/16.4);
}

//celsius
//TODO
float mpu6050_from_lsb_to_celsius(int16_t lsb)
{
//   return ((float)lsb / 16.0f + 25.0f);
    return 0;
}
