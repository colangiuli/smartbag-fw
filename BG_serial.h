//
//  BG_serial.h
//  BGserialApi
//
//  Created by Paolo Valerio on 6/25/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef BGserialApi_BG_serial_h
#define BGserialApi_BG_serial_h

#include "arduino.h"

#define BG_MT_COMMAND			0x00

#define EOPACKET	0xAA
#define BLE_HEADER_SIZE	4
#define BLE_FIELD_SIZE	2



#define BG_CLASS_ATTRIBUTE_DB		0x02

#define BG_MESSAGE_ID_WRITE         0x00
#define BG_MESSAGE_ID_READ_HANDLE	0x01
#define BG_MESSAGE_ID_READ_UUID		0x02
#define BG_MESSAGE_ID_USER_RESPONSE	0x03


#define NO_WAIT_RESPONSE 0
#define WAIT_RESPONSE 1


/************** GAP *********************/
#define BG_CLASS_GAP    0x06
#define BG_GAP_SET_MODE 0x01
 



/************* ATTRIBUTES CLIENT *************/
#define BG_CLASS_ATTRIBUTE_CLIENT	0x04

#define BG_MESSAGE_ID_WRITE_CLIENT	0x05
#define BG_MESSAGE_ID_EXECUTE_WRITE	0x0A

#define COMMIT  0
#define CANCEL  1

#define BG_MESSAGE_ID_FIND_BY_TYPE_VALUE	0x00
#define BG_MESSAGE_ID_FIND_IN_HANDLE_RANGE  0x03
#define BG_MESSAGE_ID_READ_BY_GROUP_TYPE    0x01
#define BG_MESSAGE_ID_READ_BY_HANDLE        0x04
#define BG_MESSAGE_ID_READ_BY_ATT_TYPE      0x02
#define BG_MESSAGE_ID_READ_LONG             0x08
#define BG_MESSAGE_ID_READ_MULTIPLE_HANDLES 0x0B
#define BG_MESSAGE_ID_WRITE_ATT_COMMAND     0x06


/************** CONNECTION ********************/
#define BG_CLASS_ATTRIBUTE_CONNECTION	0x03

#define BG_MESSAGE_STATUS               0x00
#define BG_MESSAGE_DISCONNECT           0x04

/************* EVENT *************************/
#define BG_PACKET_NO_EVENT -1
#define TIMEOUT_NODATA	2
#define BG_PACKET_EVENT_NOT_RECOGNIZED 0xFF
#define DEFAULT_PACKET_TIMEOUT	20


#define FORGE_FAKE_EVENT(pkt, classId, cmd)						\
						 pkt.pkt_hdr.mt_tt_hsize = 0x80;			\
						 pkt.pkt_hdr.cmd_class_id = classId;		\
						 pkt.pkt_hdr.cmd_id	= cmd


#define IS_ATTRUBUTE_REQUEST(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB)  &&         \
(pkt->pkt_hdr.cmd_id == BG_MESSAGE_ID_READ_HANDLE)

#define IS_ATTRIBUTE_WRITTEN(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB)  &&         \
(pkt->pkt_hdr.cmd_id == BG_MESSAGE_ID_WRITE )

#define IS_CONNECTION_STATUS(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_CONNECTION)  && \
(pkt->pkt_hdr.cmd_id == BG_MESSAGE_STATUS )


#define IS_CONNECTION_DISCON(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_CONNECTION)  && \
(pkt->pkt_hdr.cmd_id == BG_MESSAGE_DISCONNECT )

#define EVENT_BIT   ( 1 << 7)

#define IS_AN_EVENT(byte) ( (( byte & EVENT_BIT ) >> 7 ) == 1 )


#define RESOURCE_BUSY	-2


#define POSTPONED_EVENT	1
#define HANDLE_NOW		0




enum event_type_enum {CONNECTION_EVENT = 1, DISCONNECTION_EVENT = 2, USER_REQUEST_EVENT = 4, VALUE_NOTIFICATION_EVENT = 8 };

struct postponed_events{
	uint8_t event_type;
	char user_request_event_payload[3];
	// event_handle_value_notification no datas
	// event_handle_conn_status no datas
	// event_handle_disconnection no datas

};


//typedef unsigned char uint8_t;
/*
 struct ble_pkt_hdr{
 uint8_t message_type;
 uint8_t technology_type;
 unsigned int payload_size;
 uint8_t cmd_class_id;
 uint8_t cmd_id;
 };
 */
struct ble_pkt_hdr{
    //    uint8_t message_type;
    //    uint8_t technology_type;
    uint8_t mt_tt_hsize;
    uint8_t low_payload_size;
    uint8_t cmd_class_id;
    uint8_t cmd_id;
};

struct ble_pkt{
    struct ble_pkt_hdr pkt_hdr;
    unsigned char payload[64];
};


typedef uint8_t flag;
//typedef unsigned short uint16_t;


enum att_change_reason_en {attribute_change_reason_write_request = 0, attribute_change_reason_write_command = 1};

int reset_ble_device(uint8_t mode);
enum discoverable_mode_en {gap_non_discoverable = 0, gap_limited_discoverable = 1, gap_general_discoverable = 2, gap_broadcast = 3, gap_user_data = 4 };
enum connectable_mode_en { gap_non_connectable = 0, gap_directed_connectable = 1, gap_undirected_connectable = 2, gap_scannable_connectable = 3 };

void dbg_print_P(const char *string);
void BG_sapi_init();
int BG_sapi_read_parse(/*int fd,*/ struct ble_pkt *pkt);
int BG_sapi_read_incoming( struct ble_pkt *pkt );
int set_connection_mode_connectable_discoverable();
int write_att_request_by_handle_wo_offset(uint16_t handle, uint8_t value_len, const uint8_t* value_data, flag wait_response);
int read_attribute_by_handle_wo_offset(uint16_t handle, void *attribute, flag wait_response );
int event_handler(struct ble_pkt *incoming_packet, short postponed);

flag is_connection_established();

int get_connection_RSSI( uint8_t *rssi );



#endif
