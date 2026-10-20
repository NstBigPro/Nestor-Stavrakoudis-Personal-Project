/**
  ******************************************************************************
  * @file           : sam_m10q.h
  * @brief          : Header file for the SAM-M10Q GNSS module
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the driver function declarations and datatypes for the GNSS module.
  * Taken from the SAM-M10Q datasheet
  ******************************************************************************
**/

#ifndef SAM_M10Q_H
#define SAM_M10Q_H

#include <stdint.h>

#define GPS_INTERFACE	&huart2 // UART interface
#define GPS_HDMARX			huart2.hdmarx	// UART HDMARX


typedef enum {
	NORTH,
	EAST,
	SOUTH,
	WEST
} Cardinal_Direction;


typedef struct {
	uint8_t hour;															// UTC hour
	uint8_t minute;														// UTC minute
	uint8_t second;														// UTC second
	uint8_t centisecond;											// UTC centisecond

	uint8_t		day;														// Day
	uint8_t		month;													// Month
	uint16_t	year;														// Year

	float latitude;														// Latitude
	Cardinal_Direction latitude_direction;		// North/south
	float longitude;														// Longitude
	Cardinal_Direction longitude_direction;		// East/west

	uint8_t	fix_quality;												// Fix quality
	uint8_t	satellites;												// Number of connected satellites
	float hdop;																// HDOP

	float altitude;														// Altitude (WGS84 ellipsoid)

	float speed;																// Speed in kilometers
	float heading;															// Heading from true north

	enum {
		CHECKSUM_INVALID,
		CHECKSUM_VALID
	} checksum_validity;									// Checksum validity

} GPS_Packet;

extern volatile GPS_Packet GPS_Data;									// GPS data packet

extern uint8_t GPS_Buffer_RX[512];						// GPS receive buffer

extern volatile uint8_t GPS_Sentence[512];						// GPS current sentence


void GPS_Parse_Sentence(void);	// Sentence parser
void GPS_Parse_Byte(uint8_t byte);			// Byte parser

void GPS_New_Sentence(void);		// Callback for new sentence

void _GPS_Parse_RMC(void);	// RMC sentence parser
void _GPS_Parse_GGA(void);	// GGA sentence parser

void GPS_Init(void);				// GPS initialization



#endif
