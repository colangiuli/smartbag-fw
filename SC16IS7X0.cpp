
  #include <inttypes.h>
  #include "I2C.h"



#include "SC16IS7X0.h"
#include "Arduino.h"
//#include "Wire.h"




SC16IS7X0::SC16IS7X0(){
	UART_ADDR=0x48;
	}


void SC16IS7X0::begin(long baud, int address){
	//Serial.println("ILC BEGIN");
	I2C_init();
	//Serial.println("ILC PULLUP");
	//I2C_pullup(false);
	in = 0;
	out = 0;
	UART_ADDR = address;
	
    uint16_t divisor = 230400 / baud;
//Serial.println("0");
	I2C_write(UART_ADDR, LCR, LCR_ENABLE_DIVISOR_LATCH);
	//Serial.println("1");
	I2C_write(UART_ADDR, DLL, (uint8_t) divisor);			// low byte
	//	Serial.println("2");
	I2C_write(UART_ADDR, DLH, (uint8_t) divisor >> 8);	// high byte
//	Serial.println("3");
	I2C_write(UART_ADDR, LCR, 0xBF);  			
    I2C_write(UART_ADDR, EFR, 0xC0);  			// HW FLOW CONTROL
	
	I2C_write(UART_ADDR, LCR, 0x03);  			// 8 bits, no parity
//	Serial.println("4");
    I2C_write(UART_ADDR, FCR, 0x06);  			// fifo enable (and flush)
//	Serial.println("5");
	I2C_write(UART_ADDR, FCR, 0x01);  			// fifo enable (and flush)
//	Serial.println("6");
}

unsigned char SC16IS7X0::available_with_timeout( int timeout ) 
{

	unsigned long start_time;
	//int count = 0;
	  
	start_time = millis();
	  
	do{
	  	if (available())
			return 1;
		
	}while( millis()-start_time < timeout );

	// none found
	return 0;
}


unsigned char SC16IS7X0::available () {
    if (in != out)
        return 1;
    out = 0;
	I2C_read(UART_ADDR,RXLVL,(uint8_t)(1),&in);
    if (in == 0)
        return 0;
    if (in > sizeof SC16IS7X0_rxbuf)
        in = sizeof SC16IS7X0_rxbuf;
	I2C_read(UART_ADDR,RHR,(uint8_t)in,SC16IS7X0_rxbuf);
    return 1;
}

void SC16IS7X0::flush () {
	I2C_write(UART_ADDR, FCR, 0x07); // flush both RX and TX queues
    in = out;
}

char SC16IS7X0::read () {
	return available() ? SC16IS7X0_rxbuf[out++] : -1;
}


uint8_t SC16IS7X0::read_bin (char *buff) 
{
	//return available() ? SC16IS7X0_rxbuf[out++] : -1;

	if ( available() ){
	   *buff = SC16IS7X0_rxbuf[out++];
	   return 0;
    }

	return 1;
}


char SC16IS7X0::readBytesWithDelay(unsigned char *bufptr, uint8_t buffsize, unsigned int delay_ms)
{
	  unsigned long start_time;
	  int count = 0;
	  
	  start_time = millis();
	  
	  do{
	  	if (available()){
			bufptr[count++] = read();
		}
	  }while( millis()-start_time < delay_ms && count < buffsize );
	  
	  bufptr[count] = '\0';
}


char SC16IS7X0::readBytes (unsigned char *data, unsigned char buffsize) {
	uint8_t i, avail_bytes;

	if(!available())
		return -1;

	avail_bytes = in - out;
	if (avail_bytes > (buffsize -1))
        avail_bytes = buffsize - 1;

	for (i = 0; i <  avail_bytes ; i++){
		data[i] = SC16IS7X0_rxbuf[out++];
	}
    data[i] = 0;
	return avail_bytes;
}

unsigned char SC16IS7X0::write (unsigned char data) {
	I2C_write(UART_ADDR, THR, data);
    return 1;
}

char SC16IS7X0::writeBytes (unsigned char *data, unsigned char buffsize) {
	int i;
    int sentchar = 0;
	for (i = 0; i <  buffsize; i++){
		I2C_write(UART_ADDR, THR, data[i]);
		if (++sentchar >= 60){
		    delay(100);
            sentchar = 0;
		}else{
		    delay(10);
		}
	}
    return 1;
}	








