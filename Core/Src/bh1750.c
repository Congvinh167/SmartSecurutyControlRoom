#include "bh1750.h"

static HAL_StatusTypeDef bh1750_write_cmd(BH1750_Handle_t *dev, uint8_t cmd, uint32_t to)
{
    return HAL_I2C_Master_Transmit(dev->i2c, dev->addr, &cmd, 1, to);
}

HAL_StatusTypeDef BH1750_Init(BH1750_Handle_t *dev)
{
    HAL_StatusTypeDef r;
    r = bh1750_write_cmd(dev, BH1750_CMD_POWER_ON, 50); if (r != HAL_OK) return r;
    r = bh1750_write_cmd(dev, BH1750_CMD_RESET,    50); if (r != HAL_OK) return r;
    r = bh1750_write_cmd(dev, BH1750_CONT_HIRES,   50); if (r != HAL_OK) return r;
    HAL_Delay(180);
    return HAL_OK;
}

HAL_StatusTypeDef BH1750_ReadLux(BH1750_Handle_t *dev, float *lux, uint32_t timeout_ms)
{
    uint8_t buf[2] = {0,0};
    HAL_StatusTypeDef r = HAL_I2C_Master_Receive(dev->i2c, dev->addr, buf, 2, timeout_ms);
    if (r != HAL_OK) return r;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *lux = ((float)raw) / 1.2f;
    return HAL_OK;
}
