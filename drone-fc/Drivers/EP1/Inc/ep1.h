/**
  ******************************************************************************
  * @file           : ep1.h
  * @brief          : Header file for the Happymodel EP1 ELRS receiver
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains declarations for the CRSF ELRS parser for the Happymodel EP1 receiver.
  ******************************************************************************
**/

#ifndef EP1_H
#define EP1_H

#include <stdint.h>

#define RADIO_INTERFACE	&huart3 // UART interface
#define RADIO_HDMARX			huart3.hdmarx	// UART HDMARX

#define RADIO_SYNC_BYTE	0xC8			// CRSF sync byte (initiates sentence)
#define RADIO_RC_CHANNELS_PACKED_LENGTH 0x1A	// Frame length for RC Channels Packed
#define RADIO_RC_CHANNELS_PACKED_BYTE 0x16	// Identifying byte for RC Channels Packed

/* ELRS boundaries for axes */
#define RADIO_MIN	172
#define RADIO_CENTER	992
#define RADIO_MAX	1811

/* Roll, pitch and yaw boundaries */
#define ROLL_MAX	20		// degrees
#define PITCH_MAX 20		// degrees
#define YAW_MAX	120		// degrees/sec

typedef struct {
	uint16_t armed;
	uint16_t kill;
	uint16_t pitch;
	uint16_t roll;
	uint16_t yaw;
	uint16_t throttle;
	enum {
		CRC8_VALID,
		CRC8_INVALID
	} checksum_validity;
} Radio_Packet;

typedef struct {
	uint8_t armed:1;
	uint8_t kill:1;
	uint16_t armed_position;
	float pitch;
	float roll;
	float yaw;
	float throttle;
} Radio_Packet_Scaled;


extern volatile Radio_Packet Radio_Data;									// Radio data packet

extern volatile Radio_Packet_Scaled Radio_Scaled;					// Scaled radio data packet

extern uint8_t Radio_Buffer_RX[512];						// Radio receive buffer

extern volatile uint8_t Radio_Sentence[512];						// Radio current sentence

extern volatile uint32_t Radio_Timestamp;

void Radio_Parse_Sentence(void);	// Sentence parser
void Radio_Parse_Byte(uint8_t byte);			// Byte parser

void Radio_New_Sentence(void);		// Callback for new sentence

void _Radio_Parse_RCChannelsPacked(void);	// RC Channels Packed sentence parser

void Radio_Init(void);				// Radio initialization

#endif
