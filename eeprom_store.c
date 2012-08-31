/*
atmega328 EEPROM size 1024 byte
1 byte 0-shock / 1-open 
3 byte date
3 byte time
[2 byte shock] only for shock logs
*/

#include <avr/eeprom.h>
#include "Arduino.h"
#include "utils.h"
#include "eeprom_store.h"

#define ACCESS_DELAY 100
#define START_ADDRESS 20
#define OP_MODE_EEPROM_ADDR 2
#define FREEZE_MODE_EEPROM_ADDR 3
#define MOVE_THR_EEPROM_ADDR 4
#define SHOCK_THR_EEPROM_ADDR 6
#define RSSI_THR_EEPROM_ADDR 8
#define OPEN_THR_EEPROM_ADDR 9

extern uint8_t hour, minute, second, year, month, day;
extern uint8_t move_enabled;
extern uint8_t push_enabled;
extern uint16_t accel_thr_move;
extern uint16_t accel_thr_impact;
extern uint16_t light_thr;
extern int8_t rssi_threshold;

uint16_t current_address = START_ADDRESS;


void init_eeprom(){
     current_address = eeprom_read_word((uint16_t *)0);
     delay(ACCESS_DELAY);
     if (current_address >= 65535){
		 dbg_print_P(PSTR("INIT WITH DEF SETT."));
         current_address = START_ADDRESS;
		 eeprom_write_word((uint16_t *)0, current_address);
		 delay(ACCESS_DELAY);
		 push_enabled  = FALSE;
         move_enabled  = FALSE;
         accel_thr_move  = 300;
         accel_thr_impact  = 1000;
         rssi_threshold = -90;
         light_thr  = 100;
     }else{     
		 dbg_print_P(PSTR("READ SETT. FROM EEPROM"));
         push_enabled  = eeprom_read_byte((uint8_t *)OP_MODE_EEPROM_ADDR);
         delay(ACCESS_DELAY);
         move_enabled  = eeprom_read_byte((uint8_t *)FREEZE_MODE_EEPROM_ADDR);
         delay(ACCESS_DELAY);
         accel_thr_move  = eeprom_read_word ((uint16_t *)MOVE_THR_EEPROM_ADDR);
         delay(ACCESS_DELAY);
         accel_thr_impact  = eeprom_read_word ((uint16_t *)SHOCK_THR_EEPROM_ADDR);
         delay(ACCESS_DELAY);
         rssi_threshold =eeprom_read_byte((uint8_t *)RSSI_THR_EEPROM_ADDR);
         delay(ACCESS_DELAY);
         light_thr  = eeprom_read_word ((uint16_t *)OPEN_THR_EEPROM_ADDR);
         delay(ACCESS_DELAY);
    }
}


void store_data_event_eeprom(char kind, int16_t shock){
    if (current_address >= 1000)
        return; //we have filled the memory!!!
        
    //0-shock / 1-open 
    eeprom_write_byte((uint8_t *)current_address++, kind);
    delay(ACCESS_DELAY);
    //save year
    eeprom_write_byte((uint8_t *)current_address++, year);
    delay(ACCESS_DELAY);
    //save month
    eeprom_write_byte((uint8_t *)current_address++, month);
    delay(ACCESS_DELAY);
    //save day
    eeprom_write_byte((uint8_t *)current_address++, day);
    delay(ACCESS_DELAY);
    //save hour
    eeprom_write_byte((uint8_t *)current_address++, hour);
    delay(ACCESS_DELAY);
    //save minute
    eeprom_write_byte((uint8_t *)current_address++, minute);
    delay(ACCESS_DELAY);
    //save seconds
    eeprom_write_byte((uint8_t *)current_address++, second);  
    delay(ACCESS_DELAY);
    
    if (kind == SHOCK){
        eeprom_write_word((uint16_t *)current_address, shock);
        delay(ACCESS_DELAY);
        current_address += 2;
    }
    
    //update saved idx
    eeprom_write_word((uint16_t *)0, current_address);
    delay(ACCESS_DELAY);
}

void export_log(){
    uint16_t idx = 2;
    uint8_t tmp_hour, tmp_minute, tmp_second, tmp_year, tmp_month, tmp_day;
    char tmp_kind;
    int16_t tmp_shock;
    char tmp_message[28]; // "S 12/08/22 12:22:34 +65535"
    
    dbg_print_P(PSTR("EEPROM LOG:"));
    
    	delay(ACCESS_DELAY);
    while(idx < current_address){
        tmp_kind = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_year = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_month = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_day = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_hour = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_minute = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        tmp_second = eeprom_read_byte((uint8_t *)idx++);
        delay(ACCESS_DELAY);
        
        if (tmp_kind == SHOCK){
            tmp_shock = eeprom_read_word ((uint16_t *)idx);
            delay(ACCESS_DELAY);
            idx += 2;
            sprintf_P(tmp_message, PSTR("%c %u/%u/%u %u:%u:%u %d"), tmp_kind, tmp_year, tmp_month, tmp_day, tmp_hour, tmp_minute, tmp_second, tmp_shock);
            dbg_print(tmp_message);
        }else{
            sprintf_P(tmp_message, PSTR("%c %u/%u/%u %u:%u:%u"), tmp_kind, tmp_year, tmp_month, tmp_day, tmp_hour, tmp_minute, tmp_second);
            dbg_print(tmp_message);
        }   
    }
}

void empty_log(){
    eeprom_write_word((uint16_t *)0, 0);
    current_address = 0;
    delay(ACCESS_DELAY);
}


void eeprom_save_op_mode(){
    eeprom_write_byte((uint8_t *)OP_MODE_EEPROM_ADDR, push_enabled);
}
void eeprom_save_freeze_mode(){
    eeprom_write_byte((uint8_t *)FREEZE_MODE_EEPROM_ADDR, move_enabled);
}
void eeprom_save_move_thr(){
    eeprom_write_word((uint16_t *)MOVE_THR_EEPROM_ADDR, accel_thr_move);
}
void eeprom_save_shock_thr(){
    eeprom_write_word((uint16_t *)SHOCK_THR_EEPROM_ADDR, accel_thr_impact);
}
void eeprom_save_rssi_thr(){
    eeprom_write_byte((uint8_t *)RSSI_THR_EEPROM_ADDR, rssi_threshold);
}
void eeprom_save_open_thr(){
    eeprom_write_word((uint16_t *)OPEN_THR_EEPROM_ADDR, light_thr);
}

