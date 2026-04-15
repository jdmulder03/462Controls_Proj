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
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out);


#endif //ESET462PROJECT_MPU6050_H
