#include "arduino.h"
#include "BG_serial.h"
#include "SC16IS7X0.h"
extern struct postponed_events postponed_events_s;
extern SC16IS7X0 BG_UART;



volatile int8_t evaluated_rssi;
volatile int8_t rssi_threshold = -90;
volatile int debounce_idx, debounced_rssi;

int8_t get_rssi_threshold()
{
	return rssi_threshold;
}

uint8_t is_in_proximity()
{
	uint8_t rssi;
//	char rssi_print[30];
//	int8_t evaluated_rssi, rssi_threshold;
//	int debounce_idx, debounced_rssi;

	rssi_threshold = get_rssi_threshold();

	for(debounce_idx = 0, debounced_rssi = 0; debounce_idx < 3; debounce_idx++){
		get_connection_RSSI( &rssi );
		evaluated_rssi = rssi;// ^ 0x7F;
		debounced_rssi += evaluated_rssi;
		rssi = 0x00;

//		sprintf(rssi_print, "RSSI: %d, summ R: %d", evaluated_rssi, debounced_rssi);
//		Serial.println(rssi_print);
	}

	evaluated_rssi = debounced_rssi/debounce_idx;

	if ( evaluated_rssi > rssi_threshold )
		return 1;

	return 0;
}

/*

#define IS_ATTRUBUTE_REQUEST(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB)  &&         \
(pkt.pkt_hdr.cmd_id == BG_MESSAGE_ID_READ_HANDLE)

#define IS_ATTRIBUTE_WRITTEN(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_DB)  &&         \
(pkt.pkt_hdr.cmd_id == BG_MESSAGE_ID_WRITE )

#define IS_CONNECTION_STATUS(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_CONNECTION)  && \
(pkt.pkt_hdr.cmd_id == BG_MESSAGE_STATUS )


#define IS_CONNECTION_DISCON(pkt)                                       \
(pkt->pkt_hdr.cmd_class_id == BG_CLASS_ATTRIBUTE_CONNECTION)  && \
(pkt->pkt_hdr.cmd_id == BG_MESSAGE_DISCONNECT )

*/

void check_for_postponed_events()
{
	struct ble_pkt fake_pkt;
	
	if (  postponed_events_s.event_type & CONNECTION_EVENT ){
		memset(&fake_pkt, '\0', sizeof(fake_pkt));
		FORGE_FAKE_EVENT(fake_pkt, BG_CLASS_ATTRIBUTE_CONNECTION, BG_MESSAGE_STATUS);
		event_handler(&fake_pkt, HANDLE_NOW);	
		
		postponed_events_s.event_type &= ~(CONNECTION_EVENT);
	}
	
	if (  postponed_events_s.event_type & DISCONNECTION_EVENT ){
		memset(&fake_pkt, '\0', sizeof(fake_pkt));
		FORGE_FAKE_EVENT(fake_pkt, BG_CLASS_ATTRIBUTE_CONNECTION, BG_MESSAGE_DISCONNECT);
		event_handler(&fake_pkt, HANDLE_NOW);	
		postponed_events_s.event_type &= ~(DISCONNECTION_EVENT);	
	}

	if (  postponed_events_s.event_type & USER_REQUEST_EVENT ){
		memset(&fake_pkt, '\0', sizeof(fake_pkt));
		FORGE_FAKE_EVENT(fake_pkt, BG_CLASS_ATTRIBUTE_DB, BG_MESSAGE_ID_READ_HANDLE);
		event_handler(&fake_pkt, HANDLE_NOW);
		postponed_events_s.event_type &= ~(USER_REQUEST_EVENT);
	}

	if (  postponed_events_s.event_type & VALUE_NOTIFICATION_EVENT ){
		memset(&fake_pkt, '\0', sizeof(fake_pkt));
		FORGE_FAKE_EVENT(fake_pkt,BG_CLASS_ATTRIBUTE_DB ,BG_MESSAGE_ID_WRITE);
		memcpy( fake_pkt.payload, postponed_events_s.event_handle_value_notification, 13);	
		event_handler(&fake_pkt, HANDLE_NOW);
		postponed_events_s.event_type &= ~(VALUE_NOTIFICATION_EVENT);
	}
}



short check_for_incoming_events()
{
    struct ble_pkt incoming_packet;
    int command_status;
    
	check_for_postponed_events();
	if (!BG_UART.available())
    	return TIMEOUT_NODATA;
	
	memset(&incoming_packet, '\0', sizeof(incoming_packet));

    if ( (command_status = BG_sapi_read_incoming(&incoming_packet)) == -1){
        dbg_print_P(PSTR("An error during reading and parsing the event\n"));
        return -1;
    }
    
    if ( command_status == TIMEOUT_NODATA ){
        //dbg_print_P(PSTR("No data available\n"));
        return TIMEOUT_NODATA;
    }

	return event_handler(&incoming_packet, HANDLE_NOW);

}
