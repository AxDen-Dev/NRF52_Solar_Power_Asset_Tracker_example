#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "stdio.h"

#define BG96
//#define SARA_U2

#define QUECTEL_GPS
//#define UBLOX_GPS

#define PACKET_HEADER_SIZE 13

#define COMPANY_ID 0
#define DEVICE_TYPE 0
#define COLLECTION_CYCLE_TIMEOUT 300
#define GPS_UPDATE_TIME 90

typedef union {

	struct {

		uint8_t company_id[2];
		uint8_t device_id[2];
		uint8_t mac_address[8];
		uint8_t control_number;
		uint8_t payload[200];

	} Packet;

	uint8_t buffer[213];

} radio_packet_protocol_t;

#endif /* PROTOCOL_H_ */