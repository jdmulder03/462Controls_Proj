//
// Created by Daniel Mulder on 4/14/26.
//

#include "MPU6050.h"

//Add DMA + Interrupt pin signal chain will be cool

static HAL_StatusTypeDef MPU6050_ReadReg(I2C_HandleTypeDef* hi2c,
                                         uint8_t reg,
                                         uint8_t* data,
                                         uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef* hi2c,
                                          uint8_t reg,
                                          uint8_t* data,
                                          uint16_t len)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, data, len, 100);
}


HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    //This is a stripped down version of how Adafruit does this
    HAL_StatusTypeDef status;
    MPU6050_PwrMgmt1TypeDef pwr = {0};

    // Bus check with retries
    status = HAL_ERROR;
    for (uint8_t tries = 0; tries < MPU6050_INIT_RETRIES; tries++)
    {
        status = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR << 1, 1, 10);
        if (status == HAL_OK) break;
        HAL_Delay(10);
    }
    if (status != HAL_OK) return status;

    // WHO_AM_I sanity check
    uint8_t whoami = 0;
    status = MPU6050_ReadReg(hi2c, MPU6050_WHO_AM_I, &whoami, 1);
    if (status != HAL_OK) return status;
    if (whoami != 0x68) return HAL_ERROR;

    // Device reset
    pwr.DEV_RESET = 1;
    status = MPU6050_WriteReg(hi2c, MPU6050_PWR_MGMT_1, (uint8_t*)&pwr, 1);
    if (status != HAL_OK) return status;

    // Wait for reset bit to clea
    uint32_t tick_start = HAL_GetTick();
    do
    {
        HAL_Delay(1);
        status = MPU6050_ReadReg(hi2c, MPU6050_PWR_MGMT_1, (uint8_t*)&pwr, 1);
        if (status != HAL_OK) return status;
        if ((HAL_GetTick() - tick_start) > MPU6050_RESET_TIMEOUT_MS) return HAL_TIMEOUT;
    }
    while (pwr.DEV_RESET);

    HAL_Delay(100);

    // Signal path reset
    uint8_t sig_rst = 0x07;
    status = MPU6050_WriteReg(hi2c, MPU6050_SIG_PATH_RST, &sig_rst, 1);
    if (status != HAL_OK) return status;

    HAL_Delay(100);

    // Sample rate divisor (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    // Eventually read on interrupt potentially with DMA
    uint8_t smplrt_div = 0;
    status = MPU6050_WriteReg(hi2c, MPU6050_SMPLRT_DIV, &smplrt_div, 1);
    if (status != HAL_OK) return status;

    // DLPF bandwidth (260 Hz — filter effectively disabled)
    MPU6050_ConfigTypeDef cfg = {.DLPF_CFG = 0x03};
    status = MPU6050_WriteReg(hi2c, MPU6050_CONFIG, (uint8_t*)&cfg, 1);
    if (status != HAL_OK) return status;

    // Gyro range ±500 deg/s
    MPU6050_GyroConfigTypeDef gyro_cfg = {.FS_SEL = 0x01};
    status = MPU6050_WriteReg(hi2c, MPU6050_GYRO_CONFIG, (uint8_t*)&gyro_cfg, 1);
    if (status != HAL_OK) return status;

    // Accel range ±2g
    MPU6050_AccelConfigTypeDef accel_cfg = {.AFS_SEL = 0x00};
    status = MPU6050_WriteReg(hi2c, MPU6050_ACCEL_CONFIG, (uint8_t*)&accel_cfg, 1);
    if (status != HAL_OK) return status;

    // Clock source: PLL with gyro X reference, wake from sleep
    pwr = (MPU6050_PwrMgmt1TypeDef){
        .CLKSEL = 0x01,
        .TEMP_DIS = 0,
        .CYCLE = 0,
        .SLEEP = 0,
    };
    status = MPU6050_WriteReg(hi2c, MPU6050_PWR_MGMT_1, (uint8_t*)&pwr, 1);
    if (status != HAL_OK) return status;

    HAL_Delay(100);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out)
{
    MPU6050_RawDataTypeDef tmp;
    HAL_StatusTypeDef status = MPU6050_ReadReg(hi2c, MPU6050_ACCEL_XOUT_H,
                                               (uint8_t*)&tmp, sizeof(tmp));
    if (status != HAL_OK) return status;
    // Sensor is big-endian; swap each int16 into caller's buffer
    out->accel.x = (int16_t)__builtin_bswap16(tmp.accel.x);
    out->accel.y = (int16_t)__builtin_bswap16(tmp.accel.y);
    out->accel.z = (int16_t)__builtin_bswap16(tmp.accel.z);
    out->temp = (int16_t)__builtin_bswap16(tmp.temp);
    out->gyro.x = (int16_t)__builtin_bswap16(tmp.gyro.x);
    out->gyro.y = (int16_t)__builtin_bswap16(tmp.gyro.y);
    out->gyro.z = (int16_t)__builtin_bswap16(tmp.gyro.z);

    return HAL_OK;
}
