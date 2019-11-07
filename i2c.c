#include "i2c.h"

int InitializeI2C(void)
{
    // Software reset enabled. USCI logic held in reset state.
    UCB0CTL1 = UCSWRST;

    // Initialize all USCI registers with UCSWRST = 1 (including UCxCTL1)

    // Select USCI I2C functionality.
    UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC;
    UCB0CTL1 |= UCSSEL_2;       // SMCLK as source
    UCB0BR0 = 10; // baud rate 100kHz, therefore divider 10
    UCB0BR1 = 0;  // SMCLK 1MHz
    //UCB0TXBUF = 0;              // initialize transmit buffer to 0
    return 0;
}

/*
 * // Configure port pin to receive output from USCI B0 clock.
    SET_I2C_SCK_AS_AN_OUTPUT;
    P1SEL |= I2C_BIT + I2C_SCK_BIT;
    P1SEL2 |= I2C_BIT + I2C_SCK_BIT;

    // Configure port pin to receive output from USCI B0 MOSI.
    SET_USCIB0_MOSI_AS_AN_OUTPUT;
    P1SEL2 |= USCIB0_MOSI_BIT; P1SEL |= USCIB0_MOSI_BIT;

    // Configure port pin to receive output from USCI B0 MISO.
    SET_USCIB0_MISO_AS_AN_INPUT;
    P1SEL2 |= USCIB0_MISO_BIT; P1SEL |= USCIB0_MISO_BIT;

    // Software reset disabled. USCI logic released for operation.
    UCB0CTL1 &= ~UCSWRST;
 */

int InitializeBoard(void)
{
    P1SEL |= I2C_SDA_BIT + I2C_SCK_BIT;
    P1SEL2 |= I2C_SDA_BIT + I2C_SCK_BIT; // config for i2c

    //config.baud = 9600;

    if (InitializeI2C != 0) {
        while (1);
    }

    return 0;
}

int i2c_transfer(const struct i2c_device *dev, struct i2c_data *data)
{
    int err = 0;

    /* Set the slave device address */
    UCB0I2CSA = dev->address;

    /* Transmit data is there is any */
    if (data->tx_len > 0) {
        err = _transmit(dev, (const uint8_t *) data->tx_buf, data->tx_len);
    }

    /* Receive data is there is any */
    if ((err == 0) && (data->rx_len > 0)) {
        err = _receive(dev, (uint8_t *) data->rx_buf, data->rx_len);
    } else {
        /* No bytes to receive send the stop condition */
        UCB0CTL1 |= UCTXSTP;
    }

    return err;
}

static int _transmit(const struct i2c_device *dev, const uint8_t *buf, size_t nbytes)
{
    int err = 0;
    //IGNORE(dev);

    /* Send the start condition */
    UCB0CTL1 |= UCTR | UCTXSTT;

    /* Wait for the start condition to be sent and ready to transmit interrupt */
    while ((UCB0CTL1 & UCTXSTT) && ((IFG2 & UCB0TXIFG) == 0));

    /* Check for ACK */
    err = _check_ack(dev);

    /* If no error and bytes left to send, transmit the data */
    while ((err == 0) && (nbytes > 0)) {
        UCB0TXBUF = *buf;
        while ((IFG2 & UCB0TXIFG) == 0) {
            err = _check_ack(dev);
            if (err < 0) {
                break;
            }
        }

        buf++;
        nbytes--;
    }

    return err;
}

static int _receive(const struct i2c_device *dev, uint8_t *buf, size_t nbytes)
{
    int err = 0;
    //IGNORE(dev);

    /* Send the start and wait */
    UCB0CTL1 &= ~UCTR;
    UCB0CTL1 |= UCTXSTT;

    /* Wait for the start condition to be sent */
    while (UCB0CTL1 & UCTXSTT);

    /*
     * If there is only one byte to receive, then set the stop
     * bit as soon as start condition has been sent
     */
    if (nbytes == 1) {
        UCB0CTL1 |= UCTXSTP;
    }

    /* Check for ACK */
    err = _check_ack(dev);

    /* If no error and bytes left to receive, receive the data */
    while ((err == 0) && (nbytes > 0)) {
        /* Wait for the data */
        while ((IFG2 & UCB0RXIFG) == 0);

        *buf = UCB0RXBUF;
        buf++;
        nbytes--;

        /*
         * If there is only one byte left to receive
         * send the stop condition
         */
        if (nbytes == 1) {
            UCB0CTL1 |= UCTXSTP;
        }
    }

    return err;
}

static int _check_ack(const struct i2c_device *dev)
{
    int err = 0;
    //IGNORE(dev);

    /* Check for ACK */
    if (UCB0STAT & UCNACKIFG) {
        /* Stop the I2C transmission */
        UCB0CTL1 |= UCTXSTP;

        /* Clear the interrupt flag */
        UCB0STAT &= ~UCNACKIFG;

        /* Set the error code */
        err = -1;
    }

    return err;
}

void I2CSendByte(unsigned char SendValue)
{
    UCB0TXBUF = SendValue;
    while (UCB0STAT & UCBUSY);
}

unsigned char I2CReceiveByte()
{
    unsigned char ReceiveValue = 0;

    IFG2 &= ~UCB0RXIFG; // Clear interrupt flag.

    /* To receive data into the USCI in master mode, data must be written
     * to UCxTXBUF because receive and transmit operations operate concurrently.
     * See bottom of page 439 of Users Guide.
     */
    UCB0TXBUF = 0x00;

    while (!(IFG2 & UCB0RXIFG));
    ReceiveValue = UCB0RXBUF;

    return ReceiveValue;
}

