#include <msp430.h>
#include "LED.h"
#include "i2c.h"
#define TRUE 1

// Function prototypes
void ConfigureClockModule(void);
static int acc_read(void);
static int BLE_write(void);

int sendErr;
int recvErr;

void main(void)
{
	// Stop the watchdog timer, and configure the clock module.
	WDTCTL = WDTPW + WDTHOLD;
	ConfigureClockModule();
	    
    InitializeLEDPortPins();

    __enable_interrupt();
	// Infinite loop
  	while (TRUE)
  	{
  	    TURN_ON_GREEN_LED;
  	    TURN_ON_RED_LED;
  	    sendErr = acc_read();
  		
  		// Wait for approximately 1/4 second
  		_delay_cycles(250000);
  		
  		TURN_OFF_GREEN_LED;
  		TURN_OFF_RED_LED;
  		recvErr = BLE_write();
  		
  		// Wait for approximately 1/4 second
  		_delay_cycles(250000);
  	}
}

void ConfigureClockModule(void)
{
    // Configure Digitally Controlled Oscillator (DCO) using factory calibrations.
	DCOCTL  = CALDCO_1MHZ;
	BCSCTL1 = CALBC1_1MHZ;
}

static int acc_read(void)
{
    int err;
    struct i2c_device dev;
    struct i2c_data data;
    uint8_t rx_data[1];
    uint8_t address;

    dev.address = FIFO_READ_START;

    //address = (uint8_t) menu_read_uint("Enter the address to read: ");

    data.tx_buf = &address;
    data.tx_len = sizeof(address);
    data.rx_len = 1; //ARRAY_SIZE(rx_data);
    data.rx_buf = (uint8_t *) rx_data;

    err = i2c_transfer(&dev, &data);

    //if (err == 0) {
    //    uart_puts("\nData: ");
    //    uart_puts(_int_to_ascii(rx_data[0]));
    //    uart_putchar('\n');
    //}

    return err;
}

static int BLE_write(void)
{
    int err;
    struct i2c_device dev;
    struct i2c_data data;
    uint8_t write_cmd[2];

    dev.address = 0x50;

    write_cmd[0] = 0x00; //address to write to
    write_cmd[1] = 0x00; // data to write

    data.tx_buf = write_cmd;
    data.tx_len = 2; //ARRAY_SIZE(write_cmd);
    data.rx_len = 0;

    err = i2c_transfer(&dev, &data);

    return err;
}
