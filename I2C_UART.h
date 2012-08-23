#ifndef I2C_UART_h
#define I2C_UART_h

  #include <inttypes.h>
  #include "I2C.h"

	//SC16IS750 Register definitions
	#define THR        0x00 << 3
	#define RHR        0x00 << 3
	#define IER        0x01 << 3
	#define FCR        0x02 << 3
	#define IIR        0x02 << 3
	#define LCR        0x03 << 3
	#define MCR        0x04 << 3
	#define LSR        0x05 << 3
	#define MSR        0x06 << 3
	#define SPR        0x07 << 3
	#define TXLVL      0x08 << 3
	#define RXLVL      0x09 << 3
	#define DLAB       0x80 << 3
	#define IODIR      0x0A << 3
	#define IOSTATE    0x0B << 3
	#define IOINTMSK   0x0C << 3
	#define IOCTRL     0x0E << 3
	#define EFCR       0x0F << 3

	#define DLL        0x00 << 3
	#define DLH        0x01 << 3
	#define EFR        0x02 << 3
	#define XON1       0x04 << 3  
	#define XON2       0x05 << 3
	#define XOFF1      0x06 << 3
	#define XOFF2      0x07 << 3

	#define UART_ADDR 0x4C

	// See section 8.4 of the datasheet for definitions
	// of bits in the Line Control Register (LCR)
	#define LCR_ENABLE_DIVISOR_LATCH 0x80

	// See Chapter 11 of datasheet
	#define SPI_READ_MODE_FLAG 0x80
	
	void I2C_UART_init (long baud);
	unsigned char I2C_UART_available(void);

	unsigned char I2C_UART_read (void);
	unsigned char I2C_UART_readBytes (unsigned char *data, unsigned char buffsize);
	void I2C_UART_flush(void);
	unsigned char I2C_UART_write (unsigned char data);
	char I2C_UART_writeBytes (unsigned char *data, unsigned char buffsize);
	
#endif


