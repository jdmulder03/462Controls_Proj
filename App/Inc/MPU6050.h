//
// Created by Daniel Mulder on 4/14/26.
//

#ifndef ESET462PROJECT_MPU6050_H
#define ESET462PROJECT_MPU6050_H
#include <stdint.h>
#include "stm32g4xx_hal.h"


#define MPU6050_ADDR            0x68  // HAL wants 8-bit address (left-shifted)
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_ACCEL_XOUT_H    0x3B  // start of the 14-byte sensor block
#define MPU6050_SIG_PATH_RST    0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75

typedef struct __attribute__((packed))
{
    uint8_t CLKSEL : 3; // bits [2:0]
    uint8_t TEMP_DIS : 1; // bit 3
    uint8_t RESERVED : 1; // bit 4
    uint8_t CYCLE : 1; // bit 5
    uint8_t SLEEP : 1; // bit 6
    uint8_t DEV_RESET : 1; // bit 7
} MPU6050_PwrMgmt1TypeDef;

typedef struct __attribute__((packed))
{
    uint8_t DLPF_CFG : 3; // bits [2:0]  digital low-pass filter bandwidth
    uint8_t EXT_SYNC_SET : 3; // bits [5:3]  FSYNC pin sampling
    uint8_t RESERVED : 2; // bits [7:6]
} MPU6050_ConfigTypeDef;

typedef struct __attribute__((packed))
{
    uint8_t RESERVED : 3; // bits [2:0]
    uint8_t FS_SEL : 2; // bits [4:3]  gyro full-scale range
    uint8_t ZG_ST : 1; // bit 5        Z-axis self-test
    uint8_t YG_ST : 1; // bit 6        Y-axis self-test
    uint8_t XG_ST : 1; // bit 7        X-axis self-test
} MPU6050_GyroConfigTypeDef;

typedef struct __attribute__((packed))
{
    uint8_t RESERVED : 3; // bits [2:0]
    uint8_t AFS_SEL : 2; // bits [4:3]  accel full-scale range
    uint8_t ZA_ST : 1; // bit 5        Z-axis self-test
    uint8_t YA_ST : 1; // bit 6        Y-axis self-test
    uint8_t XA_ST : 1; // bit 7        X-axis self-test
} MPU6050_AccelConfigTypeDef;

typedef struct __attribute__((packed))
{
    uint8_t RESERVED : 1; // bit 0
    uint8_t I2C_BYPASS_EN : 1; // bit 1   expose aux I2C bus to host
    uint8_t FSYNC_INT_EN : 1; // bit 2    FSYNC pin as interrupt source
    uint8_t FSYNC_INT_LEVEL : 1; // bit 3 FSYNC active low vs high
    uint8_t INT_RD_CLEAR : 1; // bit 4    clear on any read vs status read
    uint8_t LATCH_INT_EN : 1; // bit 5    latch until cleared vs 50us pulse
    uint8_t INT_OPEN : 1; // bit 6        open-drain vs push-pull
    uint8_t INT_LEVEL : 1; // bit 7       active low vs active high
} MPU6050_IntPinCfgTypeDef;

typedef struct __attribute__((packed))
{
    uint8_t DATA_RDY_EN : 1; // bit 0     new sensor sample available
    uint8_t RESERVED1 : 2; // bits [2:1]
    uint8_t I2C_MST_INT_EN : 1; // bit 3  aux I2C master status
    uint8_t FIFO_OFLOW_EN : 1; // bit 4   FIFO overflow
    uint8_t RESERVED2 : 3; // bits [7:5]
} MPU6050_IntEnableTypeDef;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_Vec3TypeDef;

typedef struct
{
    MPU6050_Vec3TypeDef accel;
    int16_t temp;
    MPU6050_Vec3TypeDef gyro;
} MPU6050_RawDataTypeDef;

#define MPU6050_INIT_RETRIES 5
#define MPU6050_RESET_TIMEOUT_MS 100

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef MPU6050_EnableInterrupt(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out);
HAL_StatusTypeDef MPU6050_ReadAll_DMA(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out);
void MPU6050_ProcessRaw(MPU6050_RawDataTypeDef* buf);


#endif //ESET462PROJECT_MPU6050_H
