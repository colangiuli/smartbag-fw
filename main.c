#include "Arduino.h"

#include "SC16IS7X0.h"
#include "sb_services.h"
//SC16IS7X0 I2C_UART;
#include "LIS3DH.h"
#include "SIM908.h"
#include "utils.h"
#include "BG_serial.h"
#include "eeprom_store.h"






#define LIGHT_DEBOUNCE_MS 60000
#define BATTERY_UPDATE_INTERVAL 60000

int16_t lis_x;
int16_t lis_y;
int16_t lis_z;


int lightSensorPin = A0;    
int lightSensorValue = 0; 

unsigned long light_debounce_start = 0;
unsigned long battery_update_start = 0;

uint8_t move_enabled = FALSE;
uint8_t push_enabled = FALSE;
uint16_t accel_thr_move = 300;
uint16_t accel_thr_impact = 1000;
uint16_t light_thr = 100;

char received[64], numchar;

extern SC16IS7X0 BG_UART;
extern uint8_t data_2_send;
extern uint8_t hour, minute, second, year, month, day;
extern int8_t  BATT_level;
LIS3DH lis;

void setup();
void loop();

//TODO
/*
    //gestire gli stati APERTO / CHIUSO per il sensore di lucw
    
	in futuro bisogna gestire correttamente gli interrupt dei due convertitore seriale I2C
	leggere l'op mode tramite BT o comunque gestire se l'iphone mi cambia l'OP mode
*/

void setup()
{
	char tmpStr[120];
	int command_status;

	//led setting
	pinMode(RED_LED, OUTPUT); 
  	pinMode(GREEN_LED, OUTPUT);

	Serial.begin(115200);

	blinkLed(BLINK_ORANGE, 5, 500);
  	dbg_print_P(PSTR("START!"));
  	blinkLed(BLINK_GREEN, 10, 300);
	
	// enable LIS3DH
    lis.begin();        

	SIM908_init();	

	SIM908_GPS_power(OFF);
//	delay(1000);
	SIM908_GPS_power(ON);

  	dbg_print_P(PSTR("Init BG ...\n"));
  	BG_sapi_init();
  	delay(3000);

	dbg_print_P(PSTR("set_connection_mode_connectable_discoverable ...\n"));
	command_status = set_connection_mode_connectable_discoverable();
	if (command_status != 0 )
		dbg_print_P(PSTR("Error!!!\n"));
    
	dbg_print_P(PSTR("READ SETTINGS FROM EEPROM\n"));
	init_eeprom();
	
    dbg_print_P(PSTR("MAIN LOOP\n"));
}

void ble_task()
{	
	
	//char debugString[40];
    //short cmdStat;
	//uint16_t handle = 3;
    //char attribute[64];
	uint8_t not_in_proximity = 1;
	int event_status;

    //delay(1000);              // wait for a second

	if ( is_connection_established() == 1 ) {
//		get_connection_RSSI( &rssi );
		
		if( !is_in_proximity() ){
			if (write_att_request_by_handle_wo_offset(0x0017, 1, &not_in_proximity, WAIT_RESPONSE) != 0 )
				dbg_print_P(PSTR("Error writing the attribute\n"));		
		}
			
//		evaluated_rssi = rssi;// ^ 0xFF;
//		sprintf(debugString, "the RSSI is %d\n\n", evaluated_rssi);
//		Serial.println(debugString);

        //send battery data to iphone
        if (millis() - battery_update_start >= BATTERY_UPDATE_INTERVAL){
            if (write_att_request_by_handle_wo_offset(BATTERY_LEVEL_HANDLE, 1, (uint8_t *) &BATT_level, WAIT_RESPONSE) != 0 )
                dbg_print_P(PSTR("Error updating batt level\n"));
            battery_update_start = millis();
        }
	}
 
//	numchar = I2C_UART.readBytes ((unsigned char *)received, 64);
//	if (numchar > 0)
//	Serial.write((const uint8_t*)received, numchar);
//	if (BG_UART.available())
		event_status = check_for_incoming_events();

	if ( event_status == BG_PACKET_EVENT_NOT_RECOGNIZED )
		dbg_print_P(PSTR("Event not recognized\n"));


}

void LIS3DH_task()
{
		uint16_t threshold;
		
		if (move_enabled == TRUE)
			threshold = accel_thr_move;
		else
			threshold = accel_thr_impact;

		lis.getXValue(&lis_x);
		lis.getYValue(&lis_y);
		lis.getZValue(&lis_z); 

		lis_x = abs(lis_x);
		lis_y = abs(lis_y);
		lis_z = abs(lis_z);

/*
		Serial.print("acc X ");
		Serial.println(lis_x);
        Serial.print(" acc Y ");
		Serial.println(lis_y);
		Serial.print(" acc Z ");
		Serial.println(lis_z);
*/
		
        //<!-- example 50g 15/11/83 19:00 is 50 15 11 83 19 00 -->		

		if ( (lis_x > threshold) || (lis_y > threshold) || (lis_z > threshold) ){
			dbg_print_P(PSTR("THR exceded!\n"));
				int16_t max_impact = lis_x;

					if (lis_y > max_impact)
						max_impact = lis_y;
					if (lis_z > max_impact)
						max_impact = lis_z;
			

			if ( is_connection_established() == 1 )
			{
				dbg_print_P(PSTR("BT SEND ---->\n"));
				uint8_t message[7];
					//uint8_t hour, minute, second, year, month, day;
					memcpy(message, (uint8_t *)&max_impact, 2);			
					message[2] = day;
					message[3] = month;
					message[4] = year;
					message[5] = hour;
					message[6] = minute;
											
					if (move_enabled == TRUE){
					    if (write_att_request_by_handle_wo_offset(SHOCK_SERVICE_HANDLE, 7, message, WAIT_RESPONSE) != 0 )
						dbg_print_P(PSTR("Error writing shock service\n"));
					}else{
						if (write_att_request_by_handle_wo_offset(SHOCK_SERVICE_HANDLE, 7, message, WAIT_RESPONSE) != 0 )
						dbg_print_P(PSTR("Error writing shock service\n"));
					}

			}else{
			        if(push_enabled == FALSE ){
					    dbg_print_P(PSTR("gprs SEND ---->\n"));
    					if (move_enabled == TRUE)
    						sbi(data_2_send,MOVE_ALARM);
    					else
    						sbi(data_2_send,IMPACT_ALARM);
					}else{
					    dbg_print_P(PSTR("EEPROM LOG -->\n"));
					    store_data_event_eeprom(SHOCK, max_impact);
					}
			}
		}
}

void opening_detection()
{
	lightSensorValue = analogRead(lightSensorPin); 
	//Serial.print("light value is: ");
	//Serial.println(lightSensorValue);
	//return;

	if ((lightSensorValue > light_thr) && (millis() - light_debounce_start > LIGHT_DEBOUNCE_MS)) {
			dbg_print_P(PSTR("Light THR exceded!\n"));
			light_debounce_start = millis();
			if ( is_connection_established() == 1 )
			{
				dbg_print_P(PSTR("BT SEND ---->\n"));
					
					uint8_t message[6];

					memcpy(message, "1", 1);			
					message[1] = day;
					message[2] = month;
					message[3] = year;
					message[4] = hour;
					message[5] = minute;

				    if (write_att_request_by_handle_wo_offset(LIGHT_SERVICE_HANDLE, 6, message, WAIT_RESPONSE) != 0 )
						dbg_print_P(PSTR("Error writing accel z\n"));

			}else{
			        if(push_enabled == FALSE ){
					    dbg_print_P(PSTR("gprs SEND ---->\n"));
					    sbi(data_2_send,OPEN_ALARM);
			        }else{
					    dbg_print_P(PSTR("EEPROM LOG -->\n"));
					    store_data_event_eeprom(OPEN, 0);			        
			        }
			}
	}
}

uint8_t red_led_status = LOW;
uint8_t green_led_status = LOW;

void loop() 
{
	LIS3DH_task();
	//opening_detection();
	ble_task();
	SIM908_task();
	//export_log();
	if (BATT_level <= 20){
	    if (red_led_status == LOW){
	        digitalWrite(RED_LED, HIGH); 
	    }else{
	        digitalWrite(RED_LED, LOW); 
	    }  
	}
	if (green_led_status == LOW){
        digitalWrite(GREEN_LED, HIGH); 
    }else{
        digitalWrite(GREEN_LED, LOW); 
    }
	
}
