/*
 * Bit-bang I2C master — PB4/PB5 are VV55 data D4/D5.
 * Default pins: PC1=SCL, PC2=SDA.
 */
#ifndef I2C_SOFT_H
#define I2C_SOFT_H

#include "stm8s.h"

/* Board free pins: PC1=SCL, PC2=SDA (swapped vs earlier; HW I2C PB4/PB5 = D4/D5) */
#ifndef I2C_SOFT_SDA_PORT
#define I2C_SOFT_SDA_PORT  GPIOC
#define I2C_SOFT_SDA_PIN   GPIO_PIN_2
#define I2C_SOFT_SCL_PORT  GPIOC
#define I2C_SOFT_SCL_PIN   GPIO_PIN_1
#endif

void i2c_soft_init(void);
uint8_t i2c_soft_write(uint8_t addr7, const uint8_t *data, uint8_t len);
uint8_t i2c_soft_read(uint8_t addr7, uint8_t *data, uint8_t len);
uint8_t i2c_soft_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                            uint8_t *rdata, uint8_t rlen);

#endif
