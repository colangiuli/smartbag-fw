#define DEBUG 
//#include <Wire.h>
#include "SC16IS7X0.h"
#include <avr/pgmspace.h>
#include "SIM908.h"
#include "utils.h"

//TODO
/*
	implementare il parsing dei pacchetti ricevuti da GPRS
	convertire strstr in strstr_P
	gestire la ricezione dell'ok o dell'eventuale error (se non ricevo ne uno ne l'altro probabilmente il modulo e' spento)	
	migliorare poweron / poweroff del modulo gsm
	migliorare la init tenendo conto che non c'e' piu l'autobaud.
*/

SC16IS7X0 mySerial;
extern	int16_t lis_x;
extern	int16_t lis_y;
extern	int16_t lis_z;

// The time, date, location data, etc.
uint8_t hour, minute, second, year, month, day;
int32_t latitude, longitude;
uint8_t groundspeed, trackangle, fix_status;

uint8_t pwrPin = 7;
uint8_t wakeupPin = 6;

uint16_t sms_2_read = 0;
uint8_t sms_2_send = 0;
uint8_t data_2_send = 0;
uint8_t task_2_execute = 0; 

uint8_t incoming_call = FALSE;
uint8_t GPS_status = OFF;
uint8_t PWR_status = OFF;
uint8_t BATT_charging_status = NOT_CHARGING;
int8_t  BATT_level = 100;
uint8_t waiting_response = FALSE;

unsigned long startTime;

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_init                               ////
////////////////////////////////////////////////////////////////////////////

void SIM908_init(){
  int cont=0;
  int count;
  char read_buffer[100];

    pinMode(pwrPin, OUTPUT);
    pinMode(wakeupPin, OUTPUT);

    digitalWrite(wakeupPin,LOW);

	// enable I2C UART
	mySerial.begin(4800, 0x48);        
	
	//first check if there is any pending data in the buffer
	sim908_read_and_parse(); //maybe it is better to flush anything
  
	for (cont=0; cont<5; cont++){
		count = SIM908_send_at_P(PSTR("AT\r"));
		sim908_read_line(read_buffer, 100, LONG_TOUT);
		sim908_read_line(read_buffer, 100, LONG_TOUT);
    	if ( strstr(read_buffer, "OK") == NULL)
    	{
#ifdef DEBUG			
      		dbg_print_P(PSTR("NO RESP"));
#endif
			SIM908_power(ON);
			sim908_read_and_parse();
    	}
    	else{
      		//module is powered ON
#ifdef DEBUG
    		dbg_print_P(PSTR("Received AT"));
#endif		
			PWR_status = ON;
      		break;
		}
   	}
 
	 delay(5000);
	
	 //enable hw flow control
	 count = SIM908_send_at_P(PSTR("AT+IFC=2,2\r"));
     sim908_read_and_parse();
	 //disable autobaud
	 count = SIM908_send_at_P(PSTR("AT+IPR=4800\r"));
	 sim908_read_and_parse();
	 //disable command echo
	 count = SIM908_send_at_P(PSTR("ATE0\r"));
	 sim908_read_and_parse();
	 //enable caller id notification
	 count = SIM908_send_at_P(PSTR("AT+CLIP=1\r"));
	 sim908_read_and_parse();
	 // 
	 count = SIM908_send_at_P(PSTR("AT+CENG=1\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CSMP=17,167,0,241\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CLVL=90\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CMGF=1\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CNMI=2,1,0,0,0\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CREG=0\r"));
	 sim908_read_and_parse();
	 //
	 count = SIM908_send_at_P(PSTR("AT+CSCS=\"IRA\"\r"));
	 sim908_read_and_parse();
	 //slow clock (sleep mode)
	 //0-disable 1-controlled by dtr 2-when there is no data on serial, module can enter sleep mode
	 count = SIM908_send_at_P(PSTR("AT+CSCLK=0\r"));
	 sim908_read_and_parse();
 	 //
     count = SIM908_send_at_P(PSTR("AT+CGMR\r"));      
     sim908_read_and_parse();
     //
     count = SIM908_send_at_P(PSTR("AT+CGPSOUT=0\r"));      
     sim908_read_and_parse();
     //
     count = SIM908_send_at_P(PSTR("AT+CNETLIGHT=1\r"));      
     sim908_read_and_parse();
     
     //save current settings
	 count = SIM908_send_at_P(PSTR("AT&W\r"));
	 sim908_read_and_parse();
     
     startTime = millis();
     //SIM908_go_to_sleep();
     //delay(10000);
}



////////////////////////////////////////////////////////////////////////////
////				      SIM908_task                                   ////
////////////////////////////////////////////////////////////////////////////

void SIM908_task(void)
{
	char cmd[13];
    
	//first check if there is any pending data in the buffer
	if (mySerial.available())
	    sim908_read_and_parse();
	
	//there is an incoming call?
	if (incoming_call == TRUE)
	{
		//hangup
		SIM908_send_at_P(PSTR("ATH\r"));
	 	//read response (we have to wait for the OK)
		sim908_read_and_parse();
		//in future we have to check that we have received OK before setting to FALSE
		incoming_call = FALSE;
		sbi(data_2_send,POSITION);
		delay(1000);
	}
	
	//there is any sms to read?
	if (sms_2_read > 0)
	{
		//ok, we need to read them
		for(uint8_t idx = 0; idx < 16; idx++)
		{
			if(CHECKBIT(sms_2_read,idx))
			{
				//read sms
				sprintf_P(cmd,PSTR("AT+CMGR=%d\r"), idx);
				SIM908_send_at(cmd);
			 	//read response (we have to wait for the OK)
				sim908_read_and_parse();
				cbi(sms_2_read,idx);
			}//end if CHECKBIT
		}//end for
		//remove all the already read sms from memory
		SIM908_send_at_P(PSTR("AT+CMGD=1,3\r"));
	 	//read response (we have to wait for the OK)
		sim908_read_and_parse();
	}//end if sms_2_read

	//there is any sms to send?
	if (sms_2_send > 0){
		SIM908_send_data_2_sms();
	}//end if sms_2_send
	
	//there is any GPRS data to send?
	if (data_2_send > 0){
		SIM908_send_gprs_data();
	}
		
    if (millis() - startTime < TASK_INTERVAL){
        return;
    }
    //SIM908_wakeup();
    
        
	//we need to update the gps position?
	if (task_2_execute == TASK_UPDATE_GPS)
	{	
		SIM908_send_at_P(PSTR("AT+CGPSINF=32\r"));
 		//read response (we have to wait for the OK)
		sim908_read_and_parse();
		//SIM908_send_pos_2_sms("3311590142");
        task_2_execute = TASK_UPDATE_BATT;
		startTime = millis();
		return;
    }
	
	//we need to check battery status?
	if (task_2_execute == TASK_UPDATE_BATT)
	{
		SIM908_send_at_P(PSTR("AT+CBC\r"));
 		//read response (we have to wait for the OK)
		sim908_read_and_parse();
		task_2_execute = TASK_UPDATE_GPS;
		startTime = millis();
		return;
	}
	
	//we need to update the gps status?
	//if (time_to_update_gps_status)
	//{	
	//	SIM908_send_at_P(PSTR("AT+CGPSSTATUS?\r"));
 		//read response (we have to wait for the OK)
	//	sim908_read_and_parse();
	//}	
    //SIM908_go_to_sleep();		
}

////////////////////////////////////////////////////////////////////////////
////				      SIM908_go_to_sleep                           ////
////////////////////////////////////////////////////////////////////////////
void SIM908_go_to_sleep(void)
{
     SIM908_send_at_P(PSTR("AT+CSCLK=1\r"));
	 sim908_read_and_parse();
	 SIM908_send_at_P(PSTR("AT+CNETLIGHT=0\r"));      
     sim908_read_and_parse();
     digitalWrite(wakeupPin,HIGH);
}

////////////////////////////////////////////////////////////////////////////
////				      SIM908_wakeup                                 ////
////////////////////////////////////////////////////////////////////////////
void SIM908_wakeup(void)
{
     digitalWrite(wakeupPin,LOW);
     delay(100);    
     SIM908_send_at_P(PSTR("AT+CSCLK=0\r"));
	 sim908_read_and_parse();
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_at                            ////
////////////////////////////////////////////////////////////////////////////

int SIM908_send_at(char *command)
{
#ifdef DEBUG  
	  dbg_print(command);
#endif
	  mySerial.writeBytes((unsigned char*) command,  strlen(command));	
	  return 0;
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_at progmem                    ////
////////////////////////////////////////////////////////////////////////////

int SIM908_send_at_P(const char *command)
{
#ifdef DEBUG  
	  dbg_print_P(command);
#endif
	  while (pgm_read_byte(command) != 0x00){ 
	  	mySerial.write((unsigned char) pgm_read_byte(command++));	
        delay(10);
      }
      waiting_response = TRUE;
	  return 0;
}


////////////////////////////////////////////////////////////////////////////
////         	              sim908_read_and_parse                     ////
////////////////////////////////////////////////////////////////////////////

int8_t sim908_read_and_parse()
{
	char response[200];
	int8_t result, smsIdx;
	char *p_char, *p_char1, *cp;

	
	do{
		result = sim908_read_line(response, 200, SHORT_TOUT);
		
		//SMS TEXT RECEIVED?
		p_char = strstr_P(response, PSTR("+CMGR"));
		if ( p_char != NULL){
			SIM908_parse_incoming_sms(response, 200);
			continue;
		}
		
		//GPS STATUS RECEIVED?	
		p_char = strstr_P(response, PSTR("+CGPSSTATUS"));
		if ( p_char != NULL){		
			SIM908_GPS_status(response);
			continue;
		}
		
		//NEW SMS NOTIFY RECEIVED?
		p_char = strstr_P(response, PSTR("+CMTI"));		
		if ( p_char != NULL){
			cp = strstr_P(p_char, PSTR(","));
			if (cp != NULL)
			{
	            smsIdx = atoi(cp+1);
				sbi(sms_2_read,smsIdx); //set this index to be read in the bitmask
	        }
			continue;
		}
		
		//INCOMING CALL RECEIVED?	
		p_char = strstr_P(response, PSTR("+CLIP"));
		if ( p_char != NULL)
		{		
			incoming_call = TRUE;
			//TODO we need to check if the number is authorized to receive response
			continue;
		}
			
		//GPRS DATA PACKET RECEIVED?	
		p_char = strstr_P(response, PSTR("+IPD"));
		if ( p_char != NULL){		
			SIM908_handle_gprs_data(response);	
			continue;
		}
		
		//BATTERY STATUS RECEIVED?	
		p_char = strstr_P(response, PSTR("+CBC"));
		if ( p_char != NULL){		
			SIM908_parse_battery_status(response);
			continue;
		}			
		
		//MODULE SHUTDOWN RECEIVED?	
		p_char = strstr_P(response, PSTR("NORMAL POWER DOWN"));
		if ( p_char != NULL){	
			PWR_status = OFF;
			continue;
		}
						
		//MODULE POWER ON CODE RECEIVED?	
		p_char = strstr_P(response, PSTR("RDY"));
		if ( p_char != NULL){		
			PWR_status = ON;
			continue;
		}
		
		//GPS POSITION RECEIVED?	
		p_char = strstr_P(response, PSTR("32,")); //THIS MUST BE IMPROVED IN ORDER TO BE SURE THAT "0," ARE THE FIRST TWO CHARs OF THE STRING
		if ( p_char != NULL){		
			SIM908_GPS_get_position(response);
			continue;
		}
		
		//OK RECEIVED?	
		p_char = strstr_P(response, PSTR("OK")); 
		if ( p_char != NULL){
		    if (waiting_response == TRUE){ 		
                waiting_response = FALSE;
			    return SUCCESS;
		    }
		}
		//ERROR RECEIVED?	
		p_char = strstr_P(response, PSTR("ERROR")); 
		if ( p_char != NULL){		
    	    if (waiting_response == TRUE){ 		
                waiting_response = FALSE;
    		    return SUCCESS;
    	    }
		}		
									
	}while (result >= 0);
	return SUCCESS;
}

////////////////////////////////////////////////////////////////////////////
////				      parseCommand                                  ////
////////////////////////////////////////////////////////////////////////////

void parseCommand(char *cmd)
{
	    if ( strncmp(cmd, "$POS", 4) == 0){ 
			sbi(sms_2_send,POSITION);
	    }
	
}

////////////////////////////////////////////////////////////////////////////
////         		SIM908_parse_battery_status                         ////
////////////////////////////////////////////////////////////////////////////

void SIM908_parse_battery_status(char *batt_status)
{
	char *p_char, *p_char1;
	
	int8_t tmp_value = 0;
	
	//+CBC: 0,68,3842
	//+CBC: 1,81,3939 (CHARGING VIA USB)
	p_char = strstr(batt_status, "+CBC: ");
	if ( p_char == NULL){
		return;
	}
		
		
	p_char1 = strchr(p_char, ',');
	if (p_char1 == NULL){
		return;
	}
			
	*p_char1 = 0; 	
	p_char = p_char1 -1;    
	
	tmp_value = atoi(p_char);
	
	if ((tmp_value >= 0) && (tmp_value <= 2))
		BATT_charging_status = tmp_value;
	else
		return;
		
	p_char1++;	
	p_char = strchr(p_char1, ',');
	if (p_char == NULL) 
		return;
		
	*p_char = 0;	
			
	tmp_value = atoi(p_char1);	
	if ((tmp_value >= 0) && (tmp_value <= 100))
		BATT_level = tmp_value;
	else
		return;
		
#ifdef DEBUG  
	  switch(BATT_charging_status)
	  {
		case NOT_CHARGING:
					dbg_print_P(PSTR("BATT IS NOT_CHARGING"));
					break;
		case CHARGING:
					dbg_print_P(PSTR("BATT IS CHARGING"));
					break;
		case CHARGE_FINISCED:
					dbg_print_P(PSTR("BATT CHARGE FINISCED"));
					break;
	}
	dbg_print_P(PSTR("BATT LVL IS: "));
	Serial.println(BATT_level);

#endif
 			
}

////////////////////////////////////////////////////////////////////////////
////         		SIM908_handle_gprs_data                             ////
////////////////////////////////////////////////////////////////////////////

void SIM908_handle_gprs_data(char *response)
{
	//we need to remove the header
	//... code here
	
	//take the raw command
	//... code here
	
	//call parseCommand
	//... code here
}

////////////////////////////////////////////////////////////////////////////
////         		SIM908_parse_incoming_sms                           ////
////////////////////////////////////////////////////////////////////////////

int8_t SIM908_parse_incoming_sms(char *sms, uint8_t buf_size)
{
	int8_t result;
	char *p_char, *p_char1;
	char smsNumber[20];


	p_char1 = strstr(sms, "+CMGR");
	if ( p_char1 != NULL)
	{	
		//"+393311590142"
		p_char = strchr(p_char1,',') + 2;//we are on the first char of the phone number
		p_char1 = strchr(p_char, ',');
		if (p_char1 != NULL) {
			   p_char1--;
	           *p_char1 = 0; 	
	    }
		strcpy(smsNumber, p_char);
#ifdef DEBUG		
		dbg_print_P(PSTR("New SMS from: "));
		dbg_print(smsNumber);
#endif		
		//now we need to take the SMS body
		result = sim908_read_line(sms, buf_size, SHORT_TOUT);
		if (result < 0)
		{
#ifdef DEBUG			
			dbg_print_P(PSTR("Problem reading SMS body"));
#endif		
			return result; 
		}
#ifdef DEBUG			
		dbg_print_P(PSTR(" Body is: "));
		dbg_print(sms);
#endif		
		parseCommand(sms);
			
	}else
		{
#ifdef DEBUG			
			dbg_print_P(PSTR("Problem reading SMS"));
#endif			
		}
	return SUCCESS;
}


////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_sms                           ////
////////////////////////////////////////////////////////////////////////////

int SIM908_send_sms(char *text, char *number)
{
	char ctrlz[2] = {0x1a,0};
	
	SIM908_send_at_P(PSTR("AT+CMGS=\""));
	SIM908_send_at(number);
	SIM908_send_at_P(PSTR("\"\r"));
	delay(1000);

	SIM908_send_at(text);
	SIM908_send_at(ctrlz);
	return 0;
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_sms progmem                   ////
////////////////////////////////////////////////////////////////////////////

int SIM908_send_sms_P(char *text, char *number)
{
	char ctrlz[2] = {0x1a,0};

	SIM908_send_at_P(PSTR("AT+CMGS=\""));
	SIM908_send_at_P(number);
	SIM908_send_at_P(PSTR("\"\r"));
	delay(1000);

	SIM908_send_at_P(text);
	SIM908_send_at(ctrlz);
	return 0;
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_data_2_sms                    ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_data_2_sms()
{
		if(CHECKBIT(sms_2_send,MOVE_ALARM))
		{
            SIM908_send_sms_P(PSTR("MOVING!!"), PSTR("3311590142"));
		 	//read response (we have to wait for the OK)
			sim908_read_and_parse();
			cbi(sms_2_send,MOVE_ALARM);
		}//end if CHECKBIT
		
		if(CHECKBIT(sms_2_send,OPEN_ALARM))
		{
            SIM908_send_sms_P(PSTR("OPEN!!"), PSTR("3311590142"));
		 	//read response (we have to wait for the OK)
			sim908_read_and_parse();
			cbi(sms_2_send,OPEN_ALARM);
		}//end if CHECKBIT
		
		if(CHECKBIT(sms_2_send,POSITION))
		{
            SIM908_send_pos_2_sms("3311590142");
		 	//read response (we have to wait for the OK)
			sim908_read_and_parse();
			cbi(sms_2_send,POSITION);
		}//end if CHECKBIT
}


////////////////////////////////////////////////////////////////////////////
////         	              SIM908_GPS_power                          ////
////////////////////////////////////////////////////////////////////////////

void SIM908_power(uint8_t state)
{

	if ((state == ON) && (PWR_status == OFF))
	{
		// generate turn on pulse
   		digitalWrite(pwrPin, HIGH);
   		delay(1200);
   		digitalWrite(pwrPin, LOW);
   		delay(5000);

	}
	
	if ((state == OFF) && (PWR_status == ON))
	{
		SIM908_send_at_P(PSTR("AT+CPOWD=0\r"));
		sim908_read_and_parse(); //we will receive NORMAL POWER DOWN
	}

}


////////////////////////////////////////////////////////////////////////////
////         	              SIM908_GPS_power                         ////
////////////////////////////////////////////////////////////////////////////

void SIM908_GPS_power(uint8_t state)
{
	if (state == ON)
	{
		SIM908_send_at_P(PSTR("AT+CGPSPWR=1\r"));
	}else{
		SIM908_send_at_P(PSTR("AT+CGPSPWR=0\r"));
	}
	
	sim908_read_and_parse();
	
	if (state == ON)
	{
		SIM908_send_at_P(PSTR("AT+CGPSRST=1\r"));
		sim908_read_and_parse();
	}
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_GPS_status                         ////
////////////////////////////////////////////////////////////////////////////

void SIM908_GPS_status(char *status_string)
{	
	if ( strstr(status_string, "Unknown") != NULL)
	{
		GPS_status = 0;
		return;
	}
	if ( strstr(status_string, "Not") != NULL)
	{
		GPS_status = 1;
		return;
	}
	if ( strstr(status_string, "2D") != NULL)
	{
		GPS_status = 2;
		return;
	}
	if ( strstr(status_string, "3D") != NULL)
	{
		GPS_status = 3;
		return;
	}
	GPS_status = 0;
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_GPS_get_position                   ////
////////////////////////////////////////////////////////////////////////////

//0,822.961704,4038.848171,106.765625,20120516172003.000,190,9,0.000000,0.000000
//   long         lat       altitude     utc time       ttff s  speed     course
//32,150927.000,A,4038.853861,N,822.952158,E,0.00,15.05,200712,,E,A  
//    HOUR     F    LAT      D     LONG   D  sp   ang   date
  
void SIM908_GPS_get_position(char *gps_string)
{
	int32_t tmp;
	char *p_start_char; 
	char *p_dot_char;
	
	//////////////////////////////////////////////////
	// hhmmss time data
	//////////////////////////////////////////////////
	p_start_char =  strstr(gps_string, "32,") + 3; //we are on the first char of time
    if (p_start_char == NULL){
#ifdef DEBUG        
		dbg_print_P(PSTR("BAD GPS STR"));
#endif		
		return;
	}

	tmp = parsedecimal(p_start_char); 
    hour = tmp / 10000;
    minute = (tmp / 100) % 100;
    second = tmp % 100;
    
	//////////////////////////////////////////////////
	// fix_status
	//////////////////////////////////////////////////    
	p_start_char = strchr(p_start_char, ',') + 1;
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("wrong FIX val ptr"));
#endif		
		return;
	}
    if (p_start_char[0] == 'A') {
        fix_status = FIX_VALID;
    }else if (p_start_char[0] == 'V') {
        fix_status = FIX_NOT_VALID;
        return;
    }else{
#ifdef DEBUG        
        dbg_print_P(PSTR("wrong FIX val"));
#endif
        return;
    }
    
    //////////////////////////////////////////////////
    // latitude D D M M . M M M M M M
    //////////////////////////////////////////////////
	p_start_char = strchr(p_start_char, ',') + 1;
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD lat ptr"));
#endif
		return;
	}
	p_dot_char = strchr(p_start_char, '.');	
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD lat dot ptr"));
#endif
		return;
	}
	tmp = parsedecimal(p_dot_char - 2) * 1000000;
    *(p_dot_char - 2) = 0;
	tmp += parsedecimal(p_dot_char + 1);
    tmp = tmp / 60;
    tmp += parsedecimal(p_start_char) * 1000000;
    
    //////////////////////////////////////////////////
    // read latitude N/S data
    //////////////////////////////////////////////////
    
    p_start_char = strchr(p_dot_char, ',') + 1;

	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD lat sign ptr"));
#endif
		return;
	}
    
    if (p_start_char[0] == ',') {
#ifdef DEBUG        
        dbg_print_P(PSTR("wrong lat sign"));
#endif
        return;
    }
	if (p_start_char[0] == 'S') {
        tmp = tmp * -1;
    }
    if (tmp != 0)
        latitude = tmp;
        
    //////////////////////////////////////////////////
    // longitude D D D M M . M M M M M M
    //////////////////////////////////////////////////
	p_start_char = strchr(p_start_char, ',') + 1;
	
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD long ptr"));
#endif
		return;
	}	
	
	p_dot_char = strchr(p_start_char, '.');
	
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD long dot ptr"));
#endif
		return;
	}	
	
	tmp = parsedecimal(p_dot_char - 2) * 1000000;
    *(p_dot_char - 2) = 0;
	tmp += parsedecimal(p_dot_char + 1);
    tmp = tmp / 60;
    tmp += parsedecimal(p_start_char) * 1000000;
    
    //////////////////////////////////////////////////
    // read longitude E/W data
    //////////////////////////////////////////////////    
    
    p_start_char = strchr(p_dot_char, ',') + 1;
    
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD long sign ptr"));
#endif
		return;
	}

    if (p_start_char[0] == ',') {
#ifdef DEBUG        
        dbg_print_P(PSTR("wrong long sign"));
#endif
        return;
    }
	if (p_start_char[0] == 'W') {
        tmp = tmp * -1;
    }
    if (tmp != 0)
        longitude = tmp;
    //////////////////////////////////////////////////
    // groundspeed
    //////////////////////////////////////////////////
	p_start_char = strchr(p_start_char, ',')+1;

	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD speed ptr"));
#endif
		return;
	}	
	
	groundspeed = parsedecimal(p_start_char);
	
	//////////////////////////////////////////////////
	// track angle
	//////////////////////////////////////////////////
	p_start_char = strchr(p_start_char, ',')+1;
	
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD trk angle ptr"));
#endif
		return;
	}
	
	trackangle = parsedecimal(p_start_char);

    //////////////////////////////////////////////////
    // date
    //////////////////////////////////////////////////
    p_start_char = strchr(p_start_char, ',')+1;
    
	if (p_start_char == NULL){
#ifdef DEBUG	    
		dbg_print_P(PSTR("BAD date ptr"));
#endif
		return;
	}    
	
    tmp = parsedecimal(p_start_char); 
    day = tmp / 10000;
    month = (tmp / 100) % 100;
    year = tmp % 100;
	
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_print_position	                    ////
////////////////////////////////////////////////////////////////////////////

void SIM908_print_position(char *answer)
{
	sprintf_P(answer,PSTR("$POS,%ld,%ld,%d,%d"),latitude, longitude, groundspeed, trackangle);
}

////////////////////////////////////////////////////////////////////////////
////         	             SIM908_print_stats    	                    ////
////////////////////////////////////////////////////////////////////////////

char SIM908_check_status()
{
	//sprintf(answer,"$STAT,%d", status);
	return GPS_status;
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_print_date	   	                    ////
////////////////////////////////////////////////////////////////////////////

void SIM908_print_date(char *answer)
{
 	sprintf_P(answer,PSTR("$DATE,%d-%d-%d"), day, month, year);
}


////////////////////////////////////////////////////////////////////////////
////         	              SIM908_print_time	   	                    ////
////////////////////////////////////////////////////////////////////////////

void SIM908_print_time(char *answer)
{
	sprintf_P(answer,PSTR("$TIME,%d:%d:%d"), hour, minute, second);
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_cloud_send                         ////
////////////////////////////////////////////////////////////////////////////

void SIM908_cloud_send(const char* message){

	  char messageSize[4];
	  char end_c[2];
	  end_c[0]=0x1a;
	  end_c[1]='\0';
	  //char tmpBuf[270];
//	unsigned long start_time;
	
	//  char *cleaned_buffer;
	  unsigned int count = 0; 
//	  char c;
      
      //add header to packages received
	  count = SIM908_send_at_P(PSTR("AT+CIPHEAD=1\r"));
	  sim908_read_and_parse();
      count = SIM908_send_at_P(PSTR("AT+CIPSHUT\r"));
	  sim908_read_and_parse();
	  if (waiting_response == TRUE)
	    sim908_read_and_parse();
	  //set APN            
	  count = SIM908_send_at_P(PSTR("AT+CSTT=\"ibox.tim.it\", \"\",\"\"\r")); //this command can answer ok or error
	  delay(2000);
	  sim908_read_and_parse();
	  if (waiting_response == TRUE)
	    sim908_read_and_parse();
	  count = SIM908_send_at_P(PSTR("AT+CIICR\r"));
	  delay(2000);
	  sim908_read_and_parse();
	  if (waiting_response == TRUE){
          delay(3000);
	      sim908_read_and_parse();
	  }
	  count = SIM908_send_at_P(PSTR("AT+CIFSR\r")); //this command answer with the ip address or error
	  delay(2000);
	  sim908_read_and_parse();
	  if (waiting_response == TRUE){
          delay(2000);
	      sim908_read_and_parse();
	  }

	  count = SIM908_send_at_P(PSTR("AT+CIPSTART=\"TCP\",\"www.blue-chain.com\", 80\r"));
	  delay(2000);
	  sim908_read_and_parse();
	  if (waiting_response == TRUE){
          delay(2000);
	      sim908_read_and_parse();
	  }
	         
	  count = SIM908_send_at_P(PSTR("AT+CIPSEND\r"));
	  delay(2000);
	  sim908_read_and_parse();
      

	  count = SIM908_send_at_P(PSTR("POST /DataCollector.php HTTP/1.0\r\n"));
	  count = SIM908_send_at_P(PSTR("Host: blue-chain.com\r\n"));
	  count = SIM908_send_at_P(PSTR("User-Agent: Route66/0.1\r\n"));
	  count = SIM908_send_at_P(PSTR("Content-Type: application/json\r\n"));

	  count = SIM908_send_at_P(PSTR("Content-length: "));
	  itoa(strlen(message), messageSize, 10);
	  count = SIM908_send_at(messageSize);
	  count = SIM908_send_at_P(PSTR("\r\n\r\n"));
	  
	  count = SIM908_send_at((char*)message);
	  count = SIM908_send_at_P(PSTR("\n\n\n"));
      mySerial.writeBytes((unsigned char *)end_c,  1);
	  sim908_read_and_parse();
	  if (waiting_response == TRUE){
          delay(500);
	      sim908_read_and_parse();
	  }
}


////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_pos_2_cloud                   ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_pos_2_cloud()
{
    //{"imei": "013043001522278","position": ["2012-04-14 12:20:30", "dddddddd", "ddddddddd", "SSS"]}
    char position_txt[100];
    sprintf_P(position_txt,PSTR("{\"imei\": \"013043001522278\",\"position\": [[\"20%d-%d-%d %d:%d:%d\", \"%ld\", \"%ld\", \"%u\"]]}"), year, month, day, hour, minute, second, latitude, longitude, groundspeed); 
    SIM908_cloud_send(position_txt);
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_open_2_cloud                  ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_open_2_cloud()
{
    char msg_txt[100];
    sprintf_P(msg_txt,PSTR("{\"imei\": \"013043001522278\",\"opening\": [[\"20%d-%d-%d %d:%d:%d\"]]}"), year, month, day, hour, minute, second); 
    SIM908_cloud_send(msg_txt);
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_move_2_cloud                  ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_move_2_cloud()
{
    char msg_txt[100];
    sprintf_P(msg_txt,PSTR("{\"imei\": \"013043001522278\",\"move\": [[\"20%d-%d-%d %d:%d:%d\"]]}"), year, month, day, hour, minute, second); 
    SIM908_cloud_send(msg_txt);
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_impact_2_cloud                ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_impact_2_cloud()
{
    char msg_txt[100];
    sprintf_P(msg_txt,PSTR("{\"imei\": \"013043001522278\",\"impact\": [[\"20%d-%d-%d %d:%d:%d\", \"%ld\", \"%ld\", \"%ld\"]]}"), year, month, day, hour, minute, second, lis_x, lis_y, lis_z); 
    SIM908_cloud_send(msg_txt);
}




////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_gprs_data                     ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_gprs_data()
{
		//dbg_print_P(PSTR("SENDING ALARM THOUGHT GPRS\n"));

		if(CHECKBIT(data_2_send,MOVE_ALARM))
		{
            SIM908_send_move_2_cloud();
			cbi(data_2_send,MOVE_ALARM);
		}//end if CHECKBIT
		
		if(CHECKBIT(data_2_send,OPEN_ALARM))
		{
            SIM908_send_open_2_cloud();
			cbi(data_2_send,OPEN_ALARM);
		}//end if CHECKBIT
		
		if(CHECKBIT(data_2_send,POSITION))
		{
            SIM908_send_pos_2_cloud();
			cbi(data_2_send,POSITION);
		}//end if CHECKBIT

		if(CHECKBIT(data_2_send,IMPACT_ALARM))
		{
            SIM908_send_impact_2_cloud();
			cbi(data_2_send,IMPACT_ALARM);
		}//end if CHECKBIT
}

////////////////////////////////////////////////////////////////////////////
////         	              SIM908_send_pos_2_sms                     ////
////////////////////////////////////////////////////////////////////////////

void SIM908_send_pos_2_sms(char *number)
{
//http://maps.google.it/maps?q=37.771008,+-122.41175    
    char position_txt[70];
    char lat_txt[12];
    char long_txt[12];
    int dot_position = 0; 
    int i;
    
    sprintf_P(position_txt,PSTR("%ld"),latitude);
 //   dbg_print(position_txt);
          
    dot_position = strlen(position_txt);

   for (i = dot_position-1; i>= dot_position-6; i--)
      lat_txt[i+1] = position_txt[i];

       
   lat_txt[dot_position-6] = '.';
   lat_txt[dot_position+1] = '\0';
   for(i = 0; i < dot_position-6; i++)
        lat_txt[i] = position_txt[i];

    sprintf_P(position_txt,PSTR("%ld"),longitude);
    dbg_print(position_txt);
          
    dot_position = strlen(position_txt);

   for (i = dot_position-1; i>= dot_position-6; i--)
      long_txt[i+1] = position_txt[i];
       
   long_txt[dot_position-6] = '.';
   long_txt[dot_position+1] = '\0';
   for(i = 0; i < dot_position-6; i++)
        long_txt[i] = position_txt[i];  
    
    sprintf_P(position_txt,PSTR("http://maps.google.it/maps?q=%s,%s"),lat_txt, long_txt);
    
    SIM908_send_sms(position_txt, number);
}