/////////////////////////////////////////////////
// Serial port interface program               //
/////////////////////////////////////////////////

//#include <stdio.h> // standard input / output functions
//#include <string.h> // string function definitions
//#include <unistd.h> // UNIX standard function definitions
//#include <fcntl.h> // File control definitions
//#include <errno.h> // Error number definitions
//#include <termios.h> // POSIX terminal control definitions
//#include <time.h>   // time calls
//#include <inttypes.h>
//#include <stdlib.h>
//#include <errno.h>

#include "arduino.h"
#include "BG_serial.h"
#define DEBUG 1
#include "SC16IS7X0.h"
#include <avr/pgmspace.h>
#include "utils.h"
#include "eeprom_store.h"

SC16IS7X0 BG_UART;


int BG_sapi_read_parse(/*int fd,*/ struct ble_pkt *pkt);

extern uint8_t move_enabled, push_enabled;

// Packet description

/*
+---------------+-------+---------------+-------------------------------+
|		|___7___|_Message Type__|__0: Command/Response 1: Event_|
| Octet 0	|__6:3__|_Techn Type____|__0000: BT4 Single mode________|
|_______________|__2:0__|_Payload Hbits_|__Payload Length High Bits_____|
|		|	|		|		       		|
| Octet 1       |  7:0  | Payload Lbits |  Payload Length High Bits	|
|_______________|_______|_______________|_______________________________|
|		|	|		|				|
| Octet 2       |  7:0  | Class ID (CID)|  Command Class ID		|
|_______________|_______|_______________|_______________________________|
|		|	|   (CMD)	|				|
| Octet 3       |  7:0  | Command ID	|  Command ID			|
|_______________|_______|_______________|_______________________________|
|		|	|		|				|
| Octet 4-n     |   -   | Payload (PL)	|  Up to 2048 bytes		|
+-----------------------+---------------+-------------------------------+

*/
// NOTE
// The maximum allowed packet size transferred to the stack is 64 bytes, which leads to the maximum payload of 60 bytes.


struct ble_pkt incoming_pkt;
struct postponed_events postponed_events_s;

void BG_sapi_init()
{
	  char bufptr[64];
      //int bytes_a = 0;	   


	  memset(bufptr, '\0', sizeof(bufptr));

      pinMode(6, OUTPUT);
	  digitalWrite(6, HIGH);
	  BG_UART.begin(38400, 0x4C);        // enable I2C UART
	  //delay(100);
	  reset_ble_device(0);

	  //delay(10);

	memset(&postponed_events_s, '\0', sizeof(postponed_events_s));
	//postponed_events_s.event_type = 0x00;
		  
	// Here we discard the first 64 bytes into the buffer
	BG_UART.readBytesWithDelay((unsigned char *)bufptr, 64, 1000);
	// flush rx/tx queues
	BG_UART.flush();	  

	//BG_UART.readBytes((unsigned char*)bufptr, 64);
  	//delay(10);

//	  BG_UART.readBytes((unsigned char*)bufptr, 64);
//	  delay(10);

}


// Connection status
flag connected;


/************* ATTRIBUTES DATABASE *************/

// EVENTS:

// User Request
// User-backed attribute data requested

// Value
// Connected device has written to an attribute.

//The command reads the given attribute's value from the local database
// offset = NULL if we want to read the attribute's type
void BG_sapi_read(uint16_t handle, uint16_t *offset, struct ble_pkt *outgoing_packet)
{
    outgoing_packet->pkt_hdr.mt_tt_hsize 	= BG_MT_COMMAND;
    
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_READ_UUID;
    outgoing_packet->pkt_hdr.low_payload_size   = 0x02;
   
    if ( offset != NULL ){
        outgoing_packet->pkt_hdr.cmd_id		  = BG_MESSAGE_ID_READ_HANDLE;
        outgoing_packet->pkt_hdr.low_payload_size = 0x04; // 4 bytes
        outgoing_packet->payload[2]		  = (char)(*offset & 0xFF);  // Lower 8 bits
        outgoing_packet->payload[3]		  = (char)(*offset >> 8);    // uppoer 8 bits
    }

    outgoing_packet->pkt_hdr.cmd_class_id 	= BG_CLASS_ATTRIBUTE_DB;
    outgoing_packet->payload[0] 		= (char)(handle & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1]			= (char)(handle >> 8);    // uppoer 8 bits

}


void BG_sapi_user_read_request_reponse(uint8_t connection, uint8_t att_error, uint8_t value_len, const uint8_t *value_data, struct ble_pkt *outgoing_packet)
{  
 
    int payload_length_idx; 

    // CALL THE ATTRIBUTE READ BY HANDLE, WAIT FOR A RESPONSE AND GIVE BACK THE RESULT
    outgoing_packet->pkt_hdr.mt_tt_hsize	= BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_DB;
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_USER_RESPONSE;
    
    outgoing_packet->payload[0] = connection;
    outgoing_packet->payload[1] = att_error;
    
    for(payload_length_idx = 2; payload_length_idx < value_len + 2; payload_length_idx++)
	outgoing_packet->payload[payload_length_idx] = value_data[payload_length_idx];
    
    outgoing_packet->pkt_hdr.low_payload_size 	= payload_length_idx;
}


void BG_sapi_attributes_write( uint16_t handle, uint8_t offset, uint8_t value_len, const uint8_t* value_data, struct ble_pkt *outgoing_packet)
{
    int payload_length_idx;

    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_DB; 
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_WRITE;
    outgoing_packet->payload[0]                 = (char)(handle & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1]                 = (char)(handle >> 8);    // upper 8 bits

    outgoing_packet->payload[2]                 = offset;
    
    outgoing_packet->payload[3]                 = value_len;
    
    for( payload_length_idx = 4; payload_length_idx < value_len + 4; payload_length_idx++)
	{
        outgoing_packet->payload[payload_length_idx] = value_data[payload_length_idx - 4];
	}
    outgoing_packet->pkt_hdr.low_payload_size   = payload_length_idx;
}


/************* ATTRIBUTES DATABASE *************/





/************* GAP *************/

void BG_sapi_cmd_gap_set_mode(uint8_t discover, uint8_t connect, struct ble_pkt *pkt)
{
    
    pkt->pkt_hdr.mt_tt_hsize        = 0x00;
    pkt->pkt_hdr.low_payload_size   = 0x02;
    pkt->pkt_hdr.cmd_class_id       = BG_CLASS_GAP;
    pkt->pkt_hdr.cmd_id             = BG_GAP_SET_MODE;
    
    pkt->payload[0] = discover;//gap_general_discoverable;
    pkt->payload[1] = connect; //gap_undirected_connectable;
}


/************* GAP *************/

#define BG_CLASS_SYSTEM 0x00
#define BG_SYSTEM_RESET 0x00

/*********** SYSTEM *************/
void BG_sapi_cmd_system_reset(uint8_t boot_in_dfu, struct ble_pkt *pkt)
{
	pkt->pkt_hdr.mt_tt_hsize        = 0x00;
    pkt->pkt_hdr.low_payload_size   = 0x01;
    pkt->pkt_hdr.cmd_class_id       = BG_CLASS_SYSTEM;
    pkt->pkt_hdr.cmd_id             = BG_SYSTEM_RESET;
    
    pkt->payload[0] = boot_in_dfu;//gap_general_discoverable;


}



/*********** SYSTEM *************/


/********** ATTRIBUTES DB RESPONSE HANDLING *************/

int fill_linear_buffer(char *out_buffer, struct ble_pkt *pkt)
{
    short len = pkt->pkt_hdr.low_payload_size;
    len += BLE_HEADER_SIZE;
    
    (void)memcpy(out_buffer, pkt, len);
    
    return len;
}

volatile int buff_len = 0;

//TO BE ADAPTED
int serial_api_execute_command(struct ble_pkt *outgoing_packet)
{
    char out_buffer[64];
    
    
    buff_len = fill_linear_buffer(out_buffer, outgoing_packet);
    // 00 17 02 00 05 00 00 13 4d 79 20 68 65 6172742077696c6c20676f206f6e
    //return write(fd, out_buffer, buff_len);
	BG_UART.writeBytes((unsigned char*) out_buffer,  buff_len);	
	return 0;
}



short get_write_response(void *response)
{
    struct ble_pkt incoming_packet;
    //unsigned char *payload_ptr;
    short result_idx = 0;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));
    
    if (BG_sapi_read_parse(&incoming_packet) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the request\n"));
        return -1;
    }
    //payload_ptr = incoming_packet.payload;

    return (incoming_packet.payload[result_idx] | incoming_packet.payload[result_idx + 1]);
    
}

short get_read_response(void *response, uint16_t *off)
{
    struct ble_pkt incoming_packet;
    unsigned char *payload_ptr;
    short result_idx = 4;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));
    
    if (BG_sapi_read_parse(&incoming_packet) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the request\n"));
        return -1;
    }
    
    payload_ptr = incoming_packet.payload;

    if ( off == NULL ) 
        result_idx = 2;
    
    if ( (incoming_packet.payload[result_idx] | incoming_packet.payload[result_idx+1]) != 0 ){
        dbg_print_P(PSTR("An error during the request\n"));
        return (incoming_packet.payload[result_idx] | incoming_packet.payload[result_idx+1]);
    }
    
    payload_ptr += result_idx + 2;
    
    (void)memcpy(response, payload_ptr, incoming_packet.pkt_hdr.low_payload_size - result_idx);
    
    return 0;    
}

short get_att_response_response()
{
    struct ble_pkt incoming_packet;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));

    
    if (BG_sapi_read_parse(&incoming_packet) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the request\n"));
        return -1;
    }

    
    if ( incoming_packet.pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB  && 
         incoming_packet.pkt_hdr.cmd_id == BG_MESSAGE_ID_USER_RESPONSE  &&
         incoming_packet.pkt_hdr.mt_tt_hsize == BG_MT_COMMAND           &&
         incoming_packet.pkt_hdr.low_payload_size == 0x00 )
        return 0;
    
    return 1;
}



short get_set_status_response()
{
    struct ble_pkt incoming_packet;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));
    
    
    if (BG_sapi_read_parse(&incoming_packet) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the request\n"));
        return -1;
    }

    if (incoming_packet.pkt_hdr.cmd_class_id == BG_CLASS_GAP  && 
        incoming_packet.pkt_hdr.cmd_id == BG_GAP_SET_MODE     &&
        incoming_packet.pkt_hdr.mt_tt_hsize == BG_MT_COMMAND  &&
        incoming_packet.pkt_hdr.low_payload_size == 0x02      &&
        incoming_packet.payload[0] == 0x00 && incoming_packet.payload[1] == 0x00)
        return 0;

    return 1;
}




#define BG_CLASS_CONNECTION 0x03
#define BG_CONNECTION_CMD_GET_RSSI  0x01

/* Connection response handling */

short get_get_RSSI_response(uint8_t *rssi)
{
    struct ble_pkt incoming_packet;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));
    
    
    if (BG_sapi_read_parse(&incoming_packet) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the request\n"));
        return -1;
    }

    if (incoming_packet.pkt_hdr.cmd_class_id == BG_CLASS_CONNECTION  && 
        incoming_packet.pkt_hdr.cmd_id == BG_CONNECTION_CMD_GET_RSSI &&
        incoming_packet.pkt_hdr.mt_tt_hsize == BG_MT_COMMAND  		 &&
        incoming_packet.pkt_hdr.low_payload_size == 0x02     		 &&
        incoming_packet.payload[0] == 0x00){

		*rssi = incoming_packet.payload[1];
        return 0;

	}

    return 1;
}



/*************** CONNECTION API *******************/


void BG_sapi_cmd_connection_get_rssi( uint8_t connection, struct ble_pkt *pkt )
{
    pkt->pkt_hdr.mt_tt_hsize        = 0x00;
    pkt->pkt_hdr.low_payload_size   = 0x01;
    pkt->pkt_hdr.cmd_class_id       = BG_CLASS_CONNECTION;
    pkt->pkt_hdr.cmd_id             = BG_CONNECTION_CMD_GET_RSSI;
    
    pkt->payload[0] = connection;

}


/*************** CONNECTION API *******************/



/*************** EXPORTED API *********************/


// Exported API
// @param mode: 0: normal ; 1: DFU

int reset_ble_device(uint8_t mode)
{
	struct ble_pkt outgoing_packet;
	short command_status;
	//char rssi

	dbg_print_P(PSTR("resetting BLE112\n"));
	
	memset(&outgoing_packet, '\0', sizeof(outgoing_packet));

	BG_sapi_cmd_system_reset(mode, &outgoing_packet);

	command_status = serial_api_execute_command(&outgoing_packet);

   if ( command_status == -1 ){
        dbg_print_P(PSTR("Error exec reset\n"));
        return -1;
    }
	
	return command_status;

}

// Exported API
// GetRSSI
int get_connection_RSSI( uint8_t *rssi )
{
	struct ble_pkt outgoing_packet;
	short command_status;
	//char rssi

//	dbg_print_P(PSTR("getting RSSI\n"));
	
	memset(&outgoing_packet, '\0', sizeof(outgoing_packet));
	BG_sapi_cmd_connection_get_rssi( 0x00, &outgoing_packet );
	
	command_status = serial_api_execute_command(&outgoing_packet);

    if ( command_status == -1 ){
        dbg_print_P(PSTR("Error\n"));
        return -1;
    }

    command_status = get_get_RSSI_response(rssi);
    
    return command_status;


}



// Exported API
// GAP connection status
int set_connection_mode_connectable_discoverable()
{
//    enum discoverable_mode_en discoverable;
//    enum connectable_mode_en connectable;
    struct ble_pkt outgoing_packet;
    short command_status;

    memset(&outgoing_packet, '\0', sizeof(outgoing_packet));
    dbg_print_P(PSTR("gap set mode\n"));
    BG_sapi_cmd_gap_set_mode((uint8_t) gap_general_discoverable, (uint8_t) gap_undirected_connectable, &outgoing_packet);

    command_status = serial_api_execute_command(&outgoing_packet);
    
    if ( command_status == -1 ){
        dbg_print_P(PSTR("Error\n"));
        return -1;
    }
	delay(1000);
    dbg_print_P(PSTR("get set stat response\n"));
    command_status = get_set_status_response();
    
    return command_status;
    
}



// Exported API
int read_attribute_by_handle(uint16_t handle, uint16_t *offset, void *attribute, flag wait_response)
{
    struct ble_pkt outgoing_packet;
    short command_status;
    
    
    memset(&outgoing_packet, '\0', sizeof(outgoing_packet));
    
    BG_sapi_read(handle, offset, &outgoing_packet);
    
    command_status = serial_api_execute_command(&outgoing_packet);
    
    if ( command_status == -1 ){
        dbg_print_P(PSTR("Error during the execution of the command\n"));
        return -1;
    }
    
    if ( wait_response == NO_WAIT_RESPONSE )
        return command_status;
    
    command_status = get_read_response(attribute, offset);
    
    return command_status;
}

// Exported API
int read_attribute_by_handle_wo_offset(uint16_t handle, void *attribute, flag wait_response )
{
    uint16_t offset = '\0';
    
    return read_attribute_by_handle(handle, &offset, attribute, wait_response);
    
}


// Exported API
int send_reponse_to_att_request(uint8_t connection, uint16_t handle, flag wait_response)
{
    struct ble_pkt outgoing_packet;
    char attribute[60];
    uint16_t readRes;
    short command_status;
    char *attr_ptr;
    
    memset(&outgoing_packet, '\0', sizeof(outgoing_packet));
    memset(attribute, '\0', sizeof(attribute));
    
    attr_ptr = attribute;
    
    // TBD
    readRes = read_attribute_by_handle_wo_offset(handle, attribute, wait_response);
    
    attr_ptr++;    
    BG_sapi_user_read_request_reponse(connection, (uint8_t)readRes, /*strlen(attribute)*/ attribute[0], (const uint8_t *)attribute, &outgoing_packet);
    
    command_status = serial_api_execute_command(&outgoing_packet);
    
    if ( command_status == -1 ){
        dbg_print_P(PSTR("Error during the execution of the command\n"));
        return -1;
    }
    
    if ( wait_response == NO_WAIT_RESPONSE )
        return command_status;
    
    command_status = get_att_response_response();
    
    return command_status;
    
}

// Exported API
int write_att_request_by_handle(uint16_t handle, uint8_t offset, uint8_t value_len, const uint8_t* value_data, flag wait_response)
{
    struct ble_pkt outgoing_packet;
    //uint16_t readRes;
    short command_status;
    char attribute[60];

    memset(&outgoing_packet, '\0', sizeof(outgoing_packet));
    BG_sapi_attributes_write(handle, offset, value_len, value_data, &outgoing_packet);
    
    command_status = serial_api_execute_command(&outgoing_packet);
    
    if ( command_status == -1 ){
        dbg_print_P(PSTR("Error during the execution of the write command\n"));
        return -1;
    }
    
    if ( wait_response == NO_WAIT_RESPONSE )
        return command_status;
    
    command_status = get_write_response(attribute);
    
    return command_status;
}


int write_att_request_by_handle_wo_offset(uint16_t handle, uint8_t value_len, const uint8_t* value_data, flag wait_response)
{
    uint8_t offset = '\0';

    return write_att_request_by_handle(handle, offset, value_len, value_data, wait_response);
}


// User-backed attribute data requested
int event_handle_user_request(unsigned char *payload)
{
    uint16_t handle;
    uint8_t conn;
    short command_status;
    
    handle = (( payload[1] << 4 ) | payload[2] );
    //offset = (( payload[3] << 4 ) | payload[4] );
    conn   = payload[0];
    
    command_status = send_reponse_to_att_request(conn, handle, WAIT_RESPONSE); //read_attribute_by_handle(handle, &offset, attr, WAIT_RESPONSE);
    
    if (command_status != 0) {
        dbg_print_P(PSTR("Error during attribute read\n"));
        return command_status;
    }
    
    return 0;    
    //send_reponse_to_att_request(conn, handle, wait_response);
    
    
}


// Connected device has written to an attribute.
int event_handle_value_notification(unsigned char *payload, uint8_t size)
{
    uint16_t written_handle;
    
    written_handle = payload[2] + (payload[3] << 8);
        
    if ( written_handle == FREEZE_MODE_HANDLE ){
        move_enabled = payload[6];
        // Save to EEPROM
        eeprom_save_freeze_mode();
    }
    
    if ( written_handle == OP_MODE_HANDLE ) {
        push_enabled = payload[6];   
        // Save to EEPROM
        eeprom_save_op_mode();
    }
    return 0;    
    if ( written_handle == RSSI_THR_HANDLE ) {
        push_enabled = payload[6];   
        // Save to EEPROM
        eeprom_save_rssi_thr();
    }
    return 0;
    /////
    if ( written_handle == MOVE_THR_HANDLE ) {
        push_enabled = payload[6] + (payload[7] << 8);   
        // Save to EEPROM
        eeprom_save_move_thr();
    }
    return 0;
    if ( written_handle == SHOCK_THR_HANDLE ) {
        push_enabled = payload[6] + (payload[7] << 8);   
        // Save to EEPROM
        eeprom_save_shock_thr();
    }
    return 0;
    if ( written_handle == OPEN_THR_HANDLE ) {
        push_enabled = payload[6] + (payload[7] << 8);   
        // Save to EEPROM
        eeprom_save_open_thr();
    }
    return 0;
    
}

int event_handle_conn_status(unsigned char *payload)
{
	// NO ACTIONS AT THIS TIME
    return 0;    
    
}


int event_handle_disconnection(unsigned char *payload)
{
    int command_status;
    // NO ACTIONS AT THIS TIME
    command_status = set_connection_mode_connectable_discoverable();
    //query_modem();
    
    return command_status;    
}


// Exported API
int event_handler(struct ble_pkt *incoming_packet, short postponed)
{
    
    // if it is not an event return NO_EVENT
    if ( !((incoming_packet->pkt_hdr.mt_tt_hsize >> 7 ) & 1) ){
        dbg_print_P(PSTR("not an event\n"));
        return BG_PACKET_NO_EVENT;
    }
    
    
    // Client requests an attribute
    if ( IS_ATTRUBUTE_REQUEST(incoming_packet) ){
        dbg_print_P(PSTR("Attribute request event received\n"));

		if (postponed){
			postponed_events_s.event_type |= USER_REQUEST_EVENT;
			memcpy(postponed_events_s.user_request_event_payload, incoming_packet->payload, 3);

			return 0;
		}

        return event_handle_user_request(incoming_packet->payload);
    }
    
    // Client writes an attribute's value
    if ( IS_ATTRIBUTE_WRITTEN(incoming_packet) ){
		if (postponed){
			postponed_events_s.event_type |= VALUE_NOTIFICATION_EVENT;
			//memcpy(postponed_events_s.user_request_event_payload, incoming_packet.payload, 3);

			return 0;
		}

        dbg_print_P(PSTR("Attribute written event received\n"));
        return event_handle_value_notification(incoming_packet->payload, incoming_packet->pkt_hdr.low_payload_size);

    }
    
    if ( IS_CONNECTION_STATUS(incoming_packet)){

		if (postponed){
			postponed_events_s.event_type |= CONNECTION_EVENT;
			//memcpy(postponed_events_s.user_request_event_payload, incoming_packet.payload, 3);

			return 0;
		}

        dbg_print_P(PSTR("Connection status event received\n"));
		connected = 1;
        return event_handle_conn_status(incoming_packet->payload);
    
    }
    
    if ( IS_CONNECTION_DISCON(incoming_packet) ){
		if (postponed){
			postponed_events_s.event_type |= DISCONNECTION_EVENT;
			//memcpy(postponed_events_s.user_request_event_payload, incoming_packet.payload, 3);
			return 0;
		}
	
        dbg_print_P(PSTR("Disconnection event received\n"));
		connected = 0;
        return event_handle_disconnection(incoming_packet->payload);
        
    }
    
    return BG_PACKET_EVENT_NOT_RECOGNIZED;
}


/*
// Exported API
int event_handler()
{
    struct ble_pkt incoming_packet;
    int command_status;
    
    memset(&incoming_packet, '\0', sizeof(incoming_packet));

    if ( (command_status = BG_sapi_read_parse(&incoming_packet)) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the event\n"));
        return -1;
    }
    
    if ( command_status == TIMEOUT_NODATA ){
        dbg_print_P(PSTR("No data available\n"));
        return TIMEOUT_NODATA;
    }
        
    
    // if it is not an event return an error
    if ( !((incoming_packet.pkt_hdr.mt_tt_hsize >> 7 ) & 1) ){
        dbg_print_P(PSTR("the incoming packet is not an event\n"));
        return BG_PACKET_NO_EVENT;
    }
    
    
    // Client requests an attribute
    if ( IS_ATTRUBUTE_REQUEST(incoming_packet) ){
        dbg_print_P(PSTR("Attribute request event received\n"));
        return event_handle_user_request(incoming_packet.payload);
    }
    
    // Client writes an attribute's value
    if ( IS_ATTRIBUTE_WRITTEN(incoming_packet) ){
        dbg_print_P(PSTR("Attribute written event received\n"));
        return event_handle_value_notification(incoming_packet.payload);

    }
    
    if ( IS_CONNECTION_STATUS(incoming_packet)){
        dbg_print_P(PSTR("Connection status event received\n"));
		connected = 1;
        return event_handle_conn_status(incoming_packet.payload);
    
    }
    
    if ( IS_CONNECTION_DISCON(incoming_packet) ){
        dbg_print_P(PSTR("Disconnection event received\n"));
		connected = 0;
        return event_handle_disconnection(incoming_packet.payload);
        
    }
    
    return BG_PACKET_EVENT_NOT_RECOGNIZED;

}

*/ 

flag is_connection_established()
{
	if ( BG_UART.available() )
		return RESOURCE_BUSY;

	return (connected == 1);
}


/*************** EXPORTED API *********************/


//TO BE ADAPTED
int BG_sapi_read_payload(/*int fd,*/ struct ble_pkt *pkt)
{
    unsigned int byte_to_read = pkt->pkt_hdr.low_payload_size;
    int nbytes = 0;
    unsigned char *bufptr;
    
    bufptr = pkt->payload;
    
    while( byte_to_read ){
        //nbytes = read(fd, bufptr, BLE_FIELD_SIZE);
		nbytes = BG_UART.readBytes((unsigned char*)bufptr, BLE_FIELD_SIZE);
        if ( nbytes != 1 ){
            dbg_print_P(PSTR("Error reading the next byte into the payload"));
            
            return -1;
        }
        bufptr++;
        byte_to_read--;
    }
    
    return 0;
}


int BG_sapi_read_incoming( struct ble_pkt *pkt )
{
   	int read_bytes = 0;
   	char buf;
	int nbytes = 0;

	memset(pkt, '\0', sizeof(struct ble_pkt));

    while ( read_bytes != EOPACKET ){
   
        //nbytes = read(fd, bufptr, BLE_FIELD_SIZE);
        
		if ( BG_UART.read_bin(&buf) ){
           // dbg_print_P(PSTR("No Bytes available")); 
			return TIMEOUT_NODATA;
            //exit(-1);
        }
        
        //bufptr += nbytes;
        switch(read_bytes++){
                
                // 7 : message type
                // 6:3 : technology type 0000 == BT4 single mode
                // 2:0 : payload size High bits	
            case 0: 
                pkt->pkt_hdr.mt_tt_hsize = buf;
                //pkt->pkt_hdr.message_type = (( *bufptr & 0x80 ) >> 7);
                //pkt->pkt_hdr.technology_type = (( *bufptr & 0x78 ) >> 3);
                //pkt->pkt_hdr.payload_size = (( *bufptr & 0x07 ) << 8 ); // upper 3 bits
                //bufptr += nbytes;
                break;
            case 1:
                pkt->pkt_hdr.low_payload_size = buf;
                break;
            case 2:
                pkt->pkt_hdr.cmd_class_id = buf;
                break;
            case 3:
                pkt->pkt_hdr.cmd_id = buf;
                
                if ( pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB && 
                     pkt->pkt_hdr.cmd_id == BG_MESSAGE_ID_USER_RESPONSE)
                    return 0;
                
                //break;
            case 4:
                nbytes = BG_sapi_read_payload(pkt);
                if ( nbytes != -1 )
                    read_bytes = EOPACKET;
                else{
                    dbg_print_P(PSTR("Error reading the payload\n"));
                    //exit(-1);
                }
            default:
                break;
                
        }
        
	}
    
    return 0;

}


void set_postponed_event_handling( struct ble_pkt *pkt)
{
	event_handler(pkt, POSTPONED_EVENT);
	return;
}



//TO BE ADAPTED
int BG_sapi_read_parse(/*int fd,*/ struct ble_pkt *pkt)
{
	short commandStatus;    

//    if (!BG_UART.available_with_timeout(DEFAULT_PACKET_TIMEOUT)){
//    	dbg_print_P(PSTR("read Timeout!"));
// 		return TIMEOUT_NODATA;
// 	}

	
	while( BG_UART.available_with_timeout( DEFAULT_PACKET_TIMEOUT ) ) {
		commandStatus =  BG_sapi_read_incoming( pkt );
		if( IS_AN_EVENT(pkt->pkt_hdr.mt_tt_hsize) ){
			set_postponed_event_handling(pkt);
			continue;
		}

		return commandStatus;

	} 
	//dbg_print_P(PSTR("read Timeout!"));
	return TIMEOUT_NODATA;
}




/********** ATTRIBUTES DB RESPONSE HANDLING *************/



/************* ATTRIBUTES CLIENT *************/


// EVENT:
// attclient procedure completed
void BG_sapi_attclient_attribute_write(uint8_t connection, uint16_t atthandle, uint8_t data_len,const uint8_t* data_data, struct ble_pkt *outgoing_packet)
{
    int payload_length_idx;

    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT; 
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_WRITE_CLIENT;

    outgoing_packet->payload[0]                 = connection;
    outgoing_packet->payload[2]                 = (char)(atthandle & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1]                 = (char)(atthandle >> 8);    // upper 8 bits


    
    for(payload_length_idx = 3; payload_length_idx < data_len + 3; payload_length_idx++)
        outgoing_packet->payload[payload_length_idx] = data_data[payload_length_idx];
    
    outgoing_packet->pkt_hdr.low_payload_size   = payload_length_idx;
}


/*
// Executes or cancels a previously queued prepare_write commands
// EVENT:
// attclient procedure completed
void BG_sapi_attclient_execute_write(uint8_t connection, uint8_t commit, struct ble_pkt *outgoing_packet )
{
    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id		= BG_MESSAGE_ID_EXECUTE_WRITE;
    outgoing_packet->pkt_hdr.low_payload_size   = 0x02; 
    
    outgoing_packet->payload[0] = connection;
    outgoing_packet->payload[1] = commit;       
}
*/


// EVENT:
// attclient group_found
// attclient procedure completed

void BG_sapi_cmd_attclient_find_by_type_value(uint8_t connection, uint16_t start, uint16_t end, uint16_t uuid, uint8_t value_len, const uint8_t* value_data, struct ble_pkt *outgoing_packet)
{
    int payload_length_idx;

    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id		        = BG_MESSAGE_ID_FIND_BY_TYPE_VALUE;
//    outgoing_packet->pkt_hdr.low_payload_size   = 0x02; 
    outgoing_packet->payload[0] = connection;

    outgoing_packet->payload[2] = (char)(start & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1] = (char)(start >> 8);    // upper 8 bits; 
    
    outgoing_packet->payload[4] = (char)(end & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[3] = (char)(end >> 8);    // upper 8 bits; 
  
    outgoing_packet->payload[6] = (char)(uuid & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[7] = (char)(uuid >> 8);    // upper 8 bits;  

    for(payload_length_idx = 8; payload_length_idx < value_len + 8; payload_length_idx++)
        outgoing_packet->payload[payload_length_idx] = value_data[payload_length_idx];

    outgoing_packet->pkt_hdr.low_payload_size   = payload_length_idx;
}


// EVENT:
// attclient find_information_found
// attclient procedure completed
void BG_sapi_cmd_attclient_find_information(uint8_t connection, uint16_t start, uint16_t end, struct ble_pkt *outgoing_packet)
{
    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id		= BG_MESSAGE_ID_FIND_IN_HANDLE_RANGE;
    outgoing_packet->pkt_hdr.low_payload_size   = 0x05;

    outgoing_packet->payload[0] = connection;

    outgoing_packet->payload[2] = (char)(start & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1] = (char)(start >> 8);    // upper 8 bits; 
    
    outgoing_packet->payload[4] = (char)(end & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[3] = (char)(end >> 8);    // upper 8 bits; 
}

/*

// EVENT:
// attclient procedure completed
void ble_cmd_attclient_prepare_write( uint8_t connection, uint16_t atthandle, uint16_t offset, uint8_t data_len, const uint8_t* data_data, struct ble_pkt *outgoing_packet)
{

}
*/


// EVENT:
// attclient group_found 
// attclient procedure completed
void BG_sapi_cmd_attclient_read_by_group_type(uint8_t connection, uint16_t start, uint16_t end, uint8_t uuid_len, const uint8_t* uuid_data, struct ble_pkt *outgoing_packet)
{
    int payload_length_idx;

    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_READ_BY_GROUP_TYPE;
//    outgoing_packet->pkt_hdr.low_payload_size   = 0x02; 
    outgoing_packet->payload[0] = connection;

    outgoing_packet->payload[2] = (char)(start & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1] = (char)(start >> 8);    // upper 8 bits; 
    
    outgoing_packet->payload[4] = (char)(end & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[3] = (char)(end >> 8);    // upper 8 bits; 

    for(payload_length_idx = 5; payload_length_idx < uuid_len + 5; payload_length_idx++)
        outgoing_packet->payload[payload_length_idx] = uuid_data[payload_length_idx];

    outgoing_packet->pkt_hdr.low_payload_size   = payload_length_idx;
}


// EVENT:
// attclient attribute_value 
// attclient procedure completed
void BG_sapi_cmd_attclient_read_by_handle(uint8_t connection, uint16_t chrhandle, struct ble_pkt *outgoing_packet)
{
    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_READ_BY_HANDLE;
    outgoing_packet->pkt_hdr.low_payload_size   = 0x03;

    outgoing_packet->payload[0] = connection;

    outgoing_packet->payload[2] = (char)(chrhandle & 0xFF);  // Lower 8 bits
    outgoing_packet->payload[1] = (char)(chrhandle >> 8);    // upper 8 bits; 
}


// Reads the value of each attribute of a given type (UUID) and in a given attribute handle range.
// The command for example used to discover the characteristic declarations (UUID: 0x2803) of a service.
// EVENT:
// attclient attribute_value 
// attclient procedure completed
void BG_sapi_cmd_attclient_read_by_type( uint8_t connection, uint16_t start, uint16_t end, uint8_t uuid_len, const uint8_t* uuid_data, struct ble_pkt *outgoing_packet)
{
    BG_sapi_cmd_attclient_read_by_group_type( connection, start, end, uuid_len, uuid_data, outgoing_packet );
    outgoing_packet->pkt_hdr.cmd_id = BG_MESSAGE_ID_READ_BY_ATT_TYPE;
}


//Use this command to read long attribute values.
//Starts a procedure where client first sends normal read to the server. and if returned attribute value length is equal to MTU, sends read long read requests until rest of the attribute is read.
// EVENT:
// attclient attribute_value 
// attclient procedure completed
void BG_sapi_cmd_attclient_read_long( uint8_t connection, uint16_t chrhandle, struct ble_pkt *outgoing_packet)
{
    BG_sapi_cmd_attclient_read_by_handle( connection, chrhandle, outgoing_packet );

    outgoing_packet->pkt_hdr.cmd_id = BG_MESSAGE_ID_READ_LONG;
}

//Read multiple attributes from server
// EVENT:
// attclient read_multiple_response 
// attclient procedure completed
void BG_sapi_cmd_attclient_read_multiple(uint8_t connection, uint8_t handles_len, const uint8_t* handles_data, struct ble_pkt *outgoing_packet )
{
    int payload_length_idx;
    
    outgoing_packet->pkt_hdr.mt_tt_hsize        = BG_MT_COMMAND;
    outgoing_packet->pkt_hdr.cmd_class_id       = BG_CLASS_ATTRIBUTE_CLIENT;
    outgoing_packet->pkt_hdr.cmd_id             = BG_MESSAGE_ID_READ_MULTIPLE_HANDLES;
    //outgoing_packet->pkt_hdr.low_payload_size   = 0x03;

    outgoing_packet->payload[0] = connection;

    for(payload_length_idx = 1; payload_length_idx < handles_len + 1; payload_length_idx++)
        outgoing_packet->payload[payload_length_idx] = handles_data[payload_length_idx];

    outgoing_packet->pkt_hdr.low_payload_size   = payload_length_idx;

//    outgoing_packet->payload[2] = (char)(chrhandle & 0xFF);  // Lower 8 bits
//    outgoing_packet->payload[1] = (char)(chrhandle >> 8);    // upper 8 bits;     
}


void BG_sapi_cmd_attclient_write_command( uint8_t connection, uint16_t atthandle, uint8_t data_len, const uint8_t* data_data, struct ble_pkt *outgoing_packet )
{
    BG_sapi_attclient_attribute_write( connection, atthandle, data_len, data_data, outgoing_packet );
    outgoing_packet->pkt_hdr.cmd_id = BG_MESSAGE_ID_WRITE_ATT_COMMAND;
}


/*************** ATTRIBUTES CLIENT ***********/

//TO BE ADAPTED
int BG_sapi_hello()
{
  char msg[4] = {'\0', '\0', '\0', '\0'};

  msg[3] = (char)0x01;


  //return write(fd, msg, 4);
  BG_UART.writeBytes((unsigned char*) msg,  4);	 
  return 0;
}
