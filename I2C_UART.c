#include "I2C_UART.h"
#include "arduino.h"
//#include "std_include.h"

// avoid per-byte access, fill entire buffer instead to reduce I2C overhead
unsigned char i2c_uart_rxbuf[64], in, out;


void I2C_UART_init (long baud) {
	I2C_init();
	I2C_pullup(false);
	in = 0;
	out = 0;
	
    uint16_t divisor = 230400 / baud;
	I2C_write(UART_ADDR, LCR, LCR_ENABLE_DIVISOR_LATCH);
	I2C_write(UART_ADDR, DLL, divisor);			// low byte
	I2C_write(UART_ADDR, DLH, divisor >> 8);	// high byte
	
//	I2C_write(UART_ADDR, LCR, 0xBF);
//    I2C_write(UART_ADDR, EFR, 0xC0);
	 
	I2C_write(UART_ADDR, LCR, 0x03);  			// 8 bits, no parity
    I2C_write(UART_ADDR, FCR, 0x07);  			// fifo enable (and flush)
}

unsigned char I2C_UART_available () {
    if (in != out)
        return 1;
    out = 0;
	I2C_read(UART_ADDR,RXLVL,(uint8_t)(1),&in);
    if (in == 0)
        return 0;
    if (in > sizeof i2c_uart_rxbuf)
        in = sizeof i2c_uart_rxbuf;
	I2C_read(UART_ADDR,RHR,(uint8_t)in,i2c_uart_rxbuf);
    return 1;
}

unsigned char I2C_UART_read () {
    return I2C_UART_available() ? i2c_uart_rxbuf[out++] : -1;
}

unsigned char I2C_UART_readBytes (unsigned char *data, unsigned char buffsize) {
	uint8_t i, avail_bytes;

	if(!I2C_UART_available())
		return -1;

	avail_bytes = in - out;
	if (avail_bytes > (buffsize -1))
        avail_bytes = buffsize - 1;

	for (i = 0; i <  avail_bytes ; i++){
		data[i] = i2c_uart_rxbuf[out++];
	}
    data[i] = 0;
	return avail_bytes;
}

void I2C_UART_flush () {
	I2C_write(UART_ADDR, FCR, 0x07); // flush both RX and TX queues
    in = out;
}

unsigned char I2C_UART_write (unsigned char data) {
	I2C_write(UART_ADDR, THR, data);
    return 1;
}

char I2C_UART_writeBytes (unsigned char *data, unsigned char buffsize) {
	int i;
	for (i = 0; i <  buffsize; i++){
		I2C_write(UART_ADDR, THR, data[i]);
		delay(10);
	}
    return 1;
}	

