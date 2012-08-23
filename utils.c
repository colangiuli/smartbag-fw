#define DEBUG 
#include "arduino.h"
#include "SC16IS7X0.h"
#include "utils.h"
#include "SIM908.h"

extern SC16IS7X0 mySerial;

////////////////////////////////////////////////////////////////////////////
////         	                     dbg_print                          ////
////////////////////////////////////////////////////////////////////////////

void dbg_print(const char *string)
{
#ifdef DEBUG	
		Serial.print("DBG->");
		Serial.println(string);	
#endif	
}


////////////////////////////////////////////////////////////////////////////
////         	              dbg_print  progmem                        ////
////////////////////////////////////////////////////////////////////////////

void dbg_print_P(const char *string)
{
#ifdef DEBUG	
		Serial.print(F("DBG->"));
		while (pgm_read_byte(string) != 0x00) 
	  		Serial.write((unsigned char) pgm_read_byte(string++));
	
		Serial.print("\n");
#endif	
}

////////////////////////////////////////////////////////////////////////////
////         	              blinkLed					                ////
////////////////////////////////////////////////////////////////////////////

void blinkLed(uint8_t color, uint8_t repetition, uint16_t delayMs)
{
	uint8_t increment = 1;
	
	if ((color != BLINK_RED) && (color != BLINK_ORANGE) && (color != BLINK_GREEN))
		return;
	
	if (repetition == 0)
		increment = 0; //loop forever
	
	digitalWrite(RED_LED, LOW);   
	digitalWrite(GREEN_LED, LOW);   	
		
		
	for(int i = 0 ; i < repetition; i += increment)
    {
	  if ((color == BLINK_RED) || (color == BLINK_ORANGE))
      	digitalWrite(RED_LED, HIGH); 

	  if ((color == BLINK_GREEN) || (color == BLINK_ORANGE))
    	digitalWrite(GREEN_LED, HIGH);      

	  delay(delayMs); 
	
	  if ((color == BLINK_RED) || (color == BLINK_ORANGE))
      	digitalWrite(RED_LED, LOW);   

      if ((color == BLINK_GREEN) || (color == BLINK_ORANGE))
      	digitalWrite(GREEN_LED, LOW);  	  

	  delay(delayMs);
    }
	
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_read_line	                        ////
////////////////////////////////////////////////////////////////////////////

int8_t sim908_read_line(char *line_buffer, uint8_t buff_size, uint16_t tout_ms)
{
	  uint8_t count = 0;
	  unsigned long start_time;
  	 

	  if (buff_size < 2)
	  {
#ifdef DEBUG	
		  	dbg_print(PSTR("buf too small"));
#endif	
			return BUFF_TOO_SMALL; 
	  }
		
	  start_time = millis();		
		
	  do{
			if (mySerial.available())
			{
		        line_buffer[count] = mySerial.read();
				if (line_buffer[count++] == '\n')
				{
					line_buffer[count] = '\0';
#ifdef DEBUG	
					dbg_print(line_buffer);
#endif				
					return SUCCESS;
				}
		    }
		    
	  }while( ( millis()-start_time < tout_ms) && (count < (buff_size -1)) );
	
	 //TIMEOUT occurred
 	 line_buffer[count] = '\0';		
#ifdef DEBUG	
      dbg_print(line_buffer);
	  dbg_print_P(PSTR("TOUT"));
#endif				

	  return TOUT;		
}




int32_t parsedecimal(char *str) {
//unsigned int parsedecimal(char *str) {
  int32_t d = 0;
  
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}
