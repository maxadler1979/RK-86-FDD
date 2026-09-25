#include "i2c_soft.h"

/* ~50–100 kHz @ 16 MHz HSI */
static void i2c_delay(void)
{
    uint8_t i;
    for (i = 0; i < 12; i++)
        ;
}

static void sda_hi(void)
{
    I2C_SOFT_SDA_PORT->DDR &= (uint8_t)~I2C_SOFT_SDA_PIN; /* input = released OD */
}

static void sda_lo(void)
{
    I2C_SOFT_SDA_PORT->ODR &= (uint8_t)~I2C_SOFT_SDA_PIN;
    I2C_SOFT_SDA_PORT->DDR |= I2C_SOFT_SDA_PIN; /* drive low */
}

static void scl_hi(void)
{
    I2C_SOFT_SCL_PORT->DDR &= (uint8_t)~I2C_SOFT_SCL_PIN;
    /* wait stretch */
    {
        uint16_t t = 5000;
        while ((I2C_SOFT_SCL_PORT->IDR & I2C_SOFT_SCL_PIN) == 0) {
            if (--t == 0) break;
        }
    }
}

static void scl_lo(void)
{
    I2C_SOFT_SCL_PORT->ODR &= (uint8_t)~I2C_SOFT_SCL_PIN;
    I2C_SOFT_SCL_PORT->DDR |= I2C_SOFT_SCL_PIN;
}

static uint8_t sda_read(void)
{
    return (uint8_t)((I2C_SOFT_SDA_PORT->IDR & I2C_SOFT_SDA_PIN) ? 1 : 0);
}

void i2c_soft_init(void)
{
    /* HW I2C shares PB4/PB5 (our data bus) — keep it off */
    I2C->CR1 &= (uint8_t)~I2C_CR1_PE;
    CLK->PCKENR1 &= (uint8_t)~CLK_PCKENR1_I2C;

    /* Open-drain via DDR: high-Z = 1 (pull-up), DDR=1+ODR=0 = 0 */
    I2C_SOFT_SDA_PORT->CR1 &= (uint8_t)~I2C_SOFT_SDA_PIN;
    I2C_SOFT_SCL_PORT->CR1 &= (uint8_t)~I2C_SOFT_SCL_PIN;
    I2C_SOFT_SDA_PORT->CR2 &= (uint8_t)~I2C_SOFT_SDA_PIN;
    I2C_SOFT_SCL_PORT->CR2 &= (uint8_t)~I2C_SOFT_SCL_PIN;
    sda_hi();
    scl_hi();
}

static void i2c_start(void)
{
    sda_hi();
    scl_hi();
    i2c_delay();
    sda_lo();
    i2c_delay();
    scl_lo();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_lo();
    i2c_delay();
    scl_hi();
    i2c_delay();
    sda_hi();
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++) {
        if (b & 0x80)
            sda_hi();
        else
            sda_lo();
        i2c_delay();
        scl_hi();
        i2c_delay();
        scl_lo();
        b <<= 1;
    }
    sda_hi();
    i2c_delay();
    scl_hi();
    i2c_delay();
    ack = sda_read(); /* 0 = ACK */
    scl_lo();
    i2c_delay();
    return ack; /* 0 ok */
}

static uint8_t i2c_read_byte(uint8_t nack)
{
    uint8_t i;
    uint8_t b = 0;

    sda_hi();
    for (i = 0; i < 8; i++) {
        b <<= 1;
        i2c_delay();
        scl_hi();
        i2c_delay();
        if (sda_read())
            b |= 1;
        scl_lo();
    }
    if (nack)
        sda_hi();
    else
        sda_lo();
    i2c_delay();
    scl_hi();
    i2c_delay();
    scl_lo();
    sda_hi();
    i2c_delay();
    return b;
}

uint8_t i2c_soft_write(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    uint8_t i;

    i2c_start();
    if (i2c_write_byte((uint8_t)(addr7 << 1))) {
        i2c_stop();
        return 1;
    }
    for (i = 0; i < len; i++) {
        if (i2c_write_byte(data[i])) {
            i2c_stop();
            return 1;
        }
    }
    i2c_stop();
    return 0;
}

uint8_t i2c_soft_read(uint8_t addr7, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (len == 0)
        return 0;
    i2c_start();
    if (i2c_write_byte((uint8_t)((addr7 << 1) | 1))) {
        i2c_stop();
        return 1;
    }
    for (i = 0; i < len; i++)
        data[i] = i2c_read_byte((uint8_t)(i + 1 == len));
    i2c_stop();
    return 0;
}

uint8_t i2c_soft_write_read(uint8_t addr7, const uint8_t *wdata, uint8_t wlen,
                            uint8_t *rdata, uint8_t rlen)
{
    if (i2c_soft_write(addr7, wdata, wlen))
        return 1;
    return i2c_soft_read(addr7, rdata, rlen);
}
