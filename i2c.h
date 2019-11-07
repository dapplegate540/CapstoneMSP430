/*
 * i2c.h
 *
 *  Created on: Nov 6, 2019
 *      Author: dna3c
 */

#ifndef USCI_I2C_H
#define USCI_I2C_H

#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>

struct i2c_device
{
    uint8_t address;
};

struct i2c_data
{
    const void *tx_buf;
    size_t tx_len;
    void *rx_buf;
    size_t rx_len;
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * USCI B0 I2C
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * GPIO      :  P1.7
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define I2C_SDA_BIT                 BIT7
#define I2C_SDA_PORT                P1IN
#define I2C_SDA_DDR                 P1DIR
#define SET_I2C_AS_AN_INPUT         I2C_SDA_DDR &= ~I2C_SDA_BIT

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Slave Clock for GPIO Flash Memory Board
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * GPIO      :  P1.6
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define I2C_SCK_BIT                 BIT6
#define I2C_SCK_PORT                P1OUT // SPI Slave Clock output
#define I2C_SCK_DDR                 P1DIR // SPI Slave Clock direction
#define SET_I2C_SCK_AS_AN_OUTPUT    I2C_SCK_DDR |= I2C_SCK_BIT

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Accelerometer
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define FIFO_READ_START             0x28

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * BLE
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

int InitializeI2C(void);
int InitializeBoard(void);
int i2c_transfer(const struct i2c_device *dev, struct i2c_data *data);
static int _transmit(const struct i2c_device *dev, const uint8_t *buf, size_t nbytes);
static int _receive(const struct i2c_device *dev, uint8_t *buf, size_t nbytes);
static int _check_ack(const struct i2c_device *dev);
void I2CSendByte(unsigned char SendValue);
unsigned char I2CReceiveByte();

#endif
