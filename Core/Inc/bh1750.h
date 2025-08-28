#ifndef __BH1750_H__
#define __BH1750_H__

#include "stm32f4xx_hal.h"

#define BH1750_ADDR_DEFAULT   (0x23u << 1)

#define BH1750_CMD_POWER_ON   0x01
#define BH1750_CMD_RESET      0x07
#define BH1750_CONT_HIRES     0x10

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t addr;
} BH1750_Handle_t;

// API tối giản
HAL_StatusTypeDef BH1750_Init(BH1750_Handle_t *dev);
HAL_StatusTypeDef BH1750_ReadLux(BH1750_Handle_t *dev, float *lux, uint32_t timeout_ms);

#endif /* __BH1750_H__ */
