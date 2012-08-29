#ifndef SIM908_h
#define SIM908_h

#include "arduino.h"
//#include "avr_common.h"

//return values defines
#define SUCCESS 0
#define BUFF_TOO_SMALL -1
#define TOUT -2

//default values defines
#define SHORT_TOUT 2000
#define LONG_TOUT 4000

#define ON 1
#define OFF 0

#define FALSE 1
#define TRUE 0

//message 2 send defines
#define MOVE_ALARM 0
#define OPEN_ALARM 1
#define POSITION 2
#define IMPACT_ALARM 3

//battery related defines
#define NOT_CHARGING 0
#define CHARGING 1
#define CHARGE_FINISCED 2

#define FIX_VALID 1
#define FIX_NOT_VALID 0

#define TASK_INTERVAL 15000

#define TASK_UPDATE_GPS 0
#define TASK_UPDATE_BATT 1
#define TASK_SEND_GPRS_POSITION 2
#define TASK_UPDATE_TIME 3
#define TASK_UPDATE_CELL_DATA 4

//bit manipulation macros
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void SIM908_init();
void SIM908_task(void);
void SIM908_go_to_sleep(void);
void SIM908_wakeup(void);
int SIM908_send_at(char *command);
int SIM908_send_at_P(const char *command);
int8_t sim908_read_and_parse(uint16_t tout_ms);
void parseCommand(char *cmd);
void SIM908_parse_battery_status(char *batt_status);
void SIM908_handle_gprs_data(char *response);
int8_t SIM908_parse_incoming_sms(char *sms, uint8_t buf_size);
int SIM908_send_sms(char *text, char *number);
int SIM908_send_sms_P(char *text, char *number);
void SIM908_send_data_2_sms();
void SIM908_power(uint8_t state);
void SIM908_GPS_power(uint8_t state);
void SIM908_GPS_status(char *status_string);
void SIM908_GPS_get_position(char *gps_string);
void SIM908_print_position(char *answer);
char SIM908_check_status();
void SIM908_print_date(char *answer);
void SIM908_print_time(char *answer);
void SIM908_cloud_send(const char* message);
void SIM908_cloud_connect();
void SIM908_cloud_send_headers();
void SIM908_cloud_send_message(const char* message);
void SIM908_send_pos_2_cloud();
void SIM908_send_open_2_cloud();
void SIM908_send_move_2_cloud();
void SIM908_send_impact_2_cloud();
void SIM908_send_cell_data_2_cloud();
void SIM908_send_gprs_data();

void SIM908_send_pos_2_sms(char *number);
void SIM908_set_RTC_date();
void SIM908_parse_RTC_date(char *message);
void SIM908_parse_gsm_cell_data(char *message);
#endif
