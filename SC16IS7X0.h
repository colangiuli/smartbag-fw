#ifndef SC16IS7X0_h
#define SC16IS7X0_h

#include "Arduino.h"
//#include "Wire.h"
  #include <inttypes.h>

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

	//#define UART_ADDR 0x48

	// See section 8.4 of the datasheet for definitions
	// of bits in the Line Control Register (LCR)
	#define LCR_ENABLE_DIVISOR_LATCH 0x80

	// See Chapter 11 of datasheet
	#define SPI_READ_MODE_FLAG 0x80
    #define LR_MAX_TRIES 12

	#define SERIAL_BUFFER_SIZE 64

class SC16IS7X0
{
	//bool writeReg(byte addr, byte val);
	//bool readReg(byte addr, byte *val);
	
	public:
		SC16IS7X0();
		int UART_ADDR;
		unsigned char SC16IS7X0_rxbuf[SERIAL_BUFFER_SIZE];
		uint8_t in, out;
		
		void begin(long baud, int address);
		unsigned char available();
		unsigned char available_with_timeout( int timeout );
		char read();
		uint8_t read_bin (char *buff);
		unsigned char write (unsigned char data);
		char readBytes (unsigned char *data, unsigned char buffsize);
		char readBytesWithDelay(unsigned char *bufptr, uint8_t buffsize, unsigned int delay_ms);
		char writeBytes (unsigned char *data, unsigned char buffsize);
		void flush();
	//	void poll();
};

#endif
