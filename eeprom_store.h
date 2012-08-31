#ifndef eeprom_store_h
#define eeprom_store_h

#define IDX_ADDR    0
#define SHOCK       'S'
#define OPEN        'O'

void init_eeprom();
void store_data_event_eeprom(char kind, int16_t shock);
void export_log();
void empty_log();

void eeprom_save_op_mode();
void eeprom_save_freeze_mode();
void eeprom_save_move_thr();
void eeprom_save_shock_thr();
void eeprom_save_rssi_thr();
void eeprom_save_open_thr();

#endif
