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

    // Wake first — config writes (SMPLRT_DIV, DLPF, ranges) don't latch while SLEEP=1
    pwr = (MPU6050_PwrMgmt1TypeDef){
        .CLKSEL = 0x01,
        .TEMP_DIS = 0,
        .CYCLE = 0,
        .SLEEP = 0,
    };
    status = MPU6050_WriteReg(hi2c, MPU6050_PWR_MGMT_1, (uint8_t*)&pwr, 1);
    if (status != HAL_OK) return status;

    HAL_Delay(100);

    // Sample rate divisor (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    uint8_t smplrt_div = 0;
    status = MPU6050_WriteReg(hi2c, MPU6050_SMPLRT_DIV, &smplrt_div, 1);
    if (status != HAL_OK) return status;

    // DLPF bandwidth — DLPF_CFG=3 sets gyro output rate to 1kHz, 44Hz BW
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

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_EnableInterrupt(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef status;

    // Matches EXTI9_5 on PC9 configured rising-edge, no pull.
    // Originally wanted to do push-pull but MPU6050 has verified and known ringing issues
    // Active-high, 50us pulse, no latch
    MPU6050_IntPinCfgTypeDef pin_cfg = {
        .LATCH_INT_EN = 0,
        .INT_RD_CLEAR = 0,
        .INT_LEVEL = 1,
    };
    status = MPU6050_WriteReg(hi2c, MPU6050_INT_PIN_CFG, (uint8_t*)&pin_cfg, 1);
    if (status != HAL_OK) return status;

    // Fire INT pin on each new sensor sample
    MPU6050_IntEnableTypeDef int_en = {.DATA_RDY_EN = 1};
    status = MPU6050_WriteReg(hi2c, MPU6050_INT_ENABLE, (uint8_t*)&int_en, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

void MPU6050_ProcessRaw(MPU6050_RawDataTypeDef* buf)
{
    // Sensor is big-endian; swap each int16 in place
    buf->accel.x = (int16_t)__builtin_bswap16(buf->accel.x);
    buf->accel.y = (int16_t)__builtin_bswap16(buf->accel.y);
    buf->accel.z = (int16_t)__builtin_bswap16(buf->accel.z);
    buf->temp    = (int16_t)__builtin_bswap16(buf->temp);
    buf->gyro.x  = (int16_t)__builtin_bswap16(buf->gyro.x);
    buf->gyro.y  = (int16_t)__builtin_bswap16(buf->gyro.y);
    buf->gyro.z  = (int16_t)__builtin_bswap16(buf->gyro.z);
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out)
{
    HAL_StatusTypeDef status = MPU6050_ReadReg(hi2c, MPU6050_ACCEL_XOUT_H,
                                               (uint8_t*)out, sizeof(*out));
    if (status != HAL_OK) return status;
    MPU6050_ProcessRaw(out);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll_DMA(I2C_HandleTypeDef* hi2c, MPU6050_RawDataTypeDef* out)
{
    return HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR << 1, MPU6050_ACCEL_XOUT_H,
                                I2C_MEMADD_SIZE_8BIT, (uint8_t*)out, sizeof(*out));
}
