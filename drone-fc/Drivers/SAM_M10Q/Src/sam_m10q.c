/**
  ******************************************************************************
  * @file           : sam_m10q.h
  * @brief          : Header file for the SAM-M10Q GNSS module
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the driver functions of the GNSS module.
  ******************************************************************************
**/

#include "sam_m10q.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <math.h>

static uint16_t GPS_Read_Pos = 0;
static uint16_t GPS_Write_Pos = 0;
static uint16_t GPS_Sentence_Pos = 0;
static uint16_t GPS_In_Sentence = 0;

/* Extern variable declarations */
GPS_Packet GPS_Data;

uint8_t GPS_Buffer_RX[512];

uint8_t GPS_Sentence[512];

void GPS_Init(){
	/* GPS initialization */
	HAL_UART_Receive_DMA(GPS_INTERFACE,GPS_Buffer_RX,512);
	__HAL_UART_ENABLE_IT(GPS_INTERFACE,UART_IT_IDLE);		// Enable the UART idle interrupt
}

void GPS_New_Sentence(){
	/* Callback for new sentence */

	GPS_Write_Pos = 512 - __HAL_DMA_GET_COUNTER(GPS_HDMARX); // Check where DMA finished

	while(GPS_Read_Pos != GPS_Write_Pos){
		uint8_t GPS_Character = GPS_Buffer_RX[GPS_Read_Pos++];

		GPS_Parse_Byte(GPS_Character);

		if(GPS_Read_Pos >= 512){
			GPS_Read_Pos = 0;
		}
	}
}

void GPS_Parse_Byte(uint8_t byte){
	if(byte == '$'){
		GPS_Sentence_Pos = 0;
		GPS_In_Sentence = 1;
	}
	if(!GPS_In_Sentence){
		return;
	}

	GPS_Sentence[GPS_Sentence_Pos++] = byte;

	if(byte == '\n'){
		GPS_Sentence[GPS_Sentence_Pos] = '\0';
		GPS_Parse_Sentence();
		GPS_In_Sentence = 0;
	}
}
void _GPS_Parse_GGA(){
  uint8_t sentence_pos = 0;
  uint8_t buffer[32];
  uint8_t buffer_pos = 0;

  while(GPS_Sentence[sentence_pos++] != ','){} // Skip field 0 as it is the $GNGGA identifier

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 1:Obtain UTC time (hhmmss.ss)
  }
  if(buffer_pos!=0){
  buffer_pos=0;
  sentence_pos++;
  GPS_Data.hour 				= (buffer[0]-'0')*10+(buffer[1]-'0');
  GPS_Data.minute				= (buffer[2]-'0')*10+(buffer[3]-'0');
  GPS_Data.second 			= (buffer[4]-'0')*10+(buffer[5]-'0');
  GPS_Data.centisecond	= (buffer[7]-'0')*10+(buffer[8]-'0');
  }
  else{
    buffer_pos=0;
    sentence_pos++;
    GPS_Data.hour 				= 0;
    GPS_Data.minute				= 0;
    GPS_Data.second 			= 0;
    GPS_Data.centisecond	=0;
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 2:Obtain latitude (ddmm.mmmm)
  }
  uint8_t copy = 0;
  buffer[buffer_pos] = '\0';
  if(buffer_pos != 0){
    copy = 1;
  } else {
    copy = 0;
  }
  buffer_pos=0;
  sentence_pos++;
  uint8_t degrees = (buffer[0]-'0')*10+(buffer[1]-'0');
  uint8_t minutes_int = (buffer[2]-'0')*10+(buffer[3]-'0');
  uint32_t minutes_dec = 0;
  uint8_t i;
  for(i = 5; buffer[i]!='\0'; i++){
    minutes_dec += buffer[i] - '0';
    minutes_dec *= 10;
  }
  minutes_dec /= 10;

  if(copy) {
    GPS_Data.latitude = degrees + ((float) minutes_int + (float) minutes_dec / (float) pow(10,(i-5))) / 60.0f;
  } else {
    GPS_Data.latitude = 0.0f;
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 3:Obtain latitude cardinal direction
  }
  buffer_pos=0;
  sentence_pos++;
  switch(buffer[0]){
    case 'N':
      GPS_Data.latitude_direction = NORTH;
      break;
    case 'S':
      GPS_Data.latitude_direction = SOUTH;
      break;
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 4:Obtain longitude (ddmm.mmmm)
  }
  buffer[buffer_pos] = '\0';
  if(buffer_pos != 0){
    copy = 1;
  } else {
    copy = 0;
  }
  buffer_pos=0;
  sentence_pos++;
  degrees = (buffer[0]-'0')*100+(buffer[1]-'0')*10+(buffer[2]-'0');
  minutes_int = (buffer[3]-'0')*10+(buffer[4]-'0');
  minutes_dec = 0;
  for(i = 6; buffer[i]!='\0'; i++){
    minutes_dec += buffer[i] - '0';
    minutes_dec *= 10;
  }
  minutes_dec /= 10;

  if(copy) {
    GPS_Data.longitude = degrees + ((float) minutes_int + (float) minutes_dec / (float) pow(10,(i-6))) / 60.0f;
  } else {
    GPS_Data.longitude = 0.0f;
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 5:Obtain longitude cardinal direction
  }
  buffer_pos=0;
  sentence_pos++;
  switch(buffer[0]){
    case 'E':
      GPS_Data.longitude_direction = EAST;
      break;
    case 'W':
      GPS_Data.longitude_direction = WEST;
      break;
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 6:Obtain fix quality
  }
  buffer_pos=0;
  sentence_pos++;
  GPS_Data.fix_quality = buffer[0] - '0';


  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 7:Satellite number
  }
  buffer[buffer_pos] = '\0';
  buffer_pos=0;
  sentence_pos++;
  uint16_t satellites_temp = 0;
  GPS_Data.satellites = 0;
  for(i=0;buffer[i]!='\0';i++){
    satellites_temp += buffer[i] - '0';
    satellites_temp *= 10;
  }
  GPS_Data.satellites = satellites_temp / 10;

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 8:HDOP
  }
  buffer[buffer_pos] = '\0';
  buffer_pos=0;
  sentence_pos++;
  int8_t encountered_dot = -1;
  uint32_t integer = 0;
  uint32_t decimal = 0;
  for(i=0; buffer[i]!='\0'; i++){
    if(buffer[i] == '.'){
      encountered_dot = i;
      integer /= 10;
    } else {
      if(encountered_dot == -1){
        integer += buffer[i] - '0';
        integer *= 10;
      }
      else {
        decimal += buffer[i] - '0';
        decimal *= 10;
      }
    }
  }
  decimal /= 10;
  if(encountered_dot == -1){
    GPS_Data.hdop = (float) integer / 10.0f;
  }
  else{
    GPS_Data.hdop = (float) integer + (float) decimal / (float) pow(10,i-encountered_dot-1);
  }

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 9:Altitude (mean sea level)
  }
  buffer[buffer_pos] = '\0';
  buffer_pos=0;
  sentence_pos++;
  encountered_dot = -1;
  decimal = 0;
  integer = 0;
  int8_t sign = (buffer[0]=='-')?-1:1;
  for(i=((sign==-1)?1:0); buffer[i]!='\0'; i++){
    if(buffer[i] == '.'){
      encountered_dot = i;
      integer /= 10;
    } else {
      if(encountered_dot == -1){
        integer += buffer[i] - '0';
        integer *= 10;
      }
      else {
        decimal += buffer[i] - '0';
        decimal *= 10;
      }
    }
  }
  decimal /= 10;
  if(encountered_dot == -1){
    GPS_Data.altitude = (float) integer / 10.0f;
  }
  else{
    GPS_Data.altitude = (float) integer + (float) decimal / (float) pow(10,i-encountered_dot-1);
  }
  GPS_Data.altitude *= sign;

  while(GPS_Sentence[sentence_pos++] != ','){} // Skip field 10 as it is always meters

  while(GPS_Sentence[sentence_pos] != ','){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 11:WGS84 Ellipsoid Correction
  }
  buffer[buffer_pos] = '\0';
  buffer_pos=0;
  sentence_pos++;
  encountered_dot = -1;
  decimal = 0;
  integer = 0;
  sign = (buffer[0]=='-')?-1:1;
  float correction;
  for(i=((sign==-1)?1:0); buffer[i]!='\0'; i++){
    if(buffer[i] == '.'){
      encountered_dot = i;
      integer /= 10;
    } else {
      if(encountered_dot == -1){
        integer += buffer[i] - '0';
        integer *= 10;
      }
      else {
        decimal += buffer[i] - '0';
        decimal *= 10;
      }
    }
  }
  decimal /= 10;
  if(encountered_dot == -1){
    correction = (float) integer / 10.0f;
  }
  else{
    correction = (float) integer + (float) decimal / (float) pow(10,i-encountered_dot-1);
  }
  correction *= sign;
  GPS_Data.altitude += correction; // Apply WGS84 ellipsoid correction

  while(GPS_Sentence[sentence_pos++] != ','){}
  sentence_pos++; // Skip field 12 as it is always meters


  while(GPS_Sentence[sentence_pos++] != ','){}
  sentence_pos++; // Skip field 13 as it is irrelevant


  while(GPS_Sentence[sentence_pos++] != '*'){}// Skip field 14 as it is irrelevant
  uint8_t checksum_end = sentence_pos - 1; // Save the position of the '*' character as it is a checksum delimeter


  while(GPS_Sentence[sentence_pos] != '\r'){
    buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Field 15:Checksum
  }
  buffer_pos=0;
  sentence_pos=1; // Prepare for checksum from position 1 (exclude the $)
  uint8_t checksum_provided_H = (buffer[0] -'0');
  if(checksum_provided_H > 9){
    checksum_provided_H -= 7;
  }
  uint8_t checksum_provided_L = (buffer[1] -'0');
  if(checksum_provided_L > 9){
    checksum_provided_L -= 7;
  }

  uint8_t checksum_provided = checksum_provided_H * 16 + checksum_provided_L;
  uint8_t checksum_calculated = 0x00;
  for(sentence_pos=1;sentence_pos<checksum_end;sentence_pos++){
    checksum_calculated ^= GPS_Sentence[sentence_pos];
  }

  if(checksum_calculated == checksum_provided){
    GPS_Data.checksum_validity = CHECKSUM_VALID;
  } else {
    GPS_Data.checksum_validity = CHECKSUM_INVALID;
  }
}

void _GPS_Parse_RMC(){
	uint8_t sentence_pos = 0;
	uint8_t buffer[32];
	uint8_t buffer_pos = 0;

	while(GPS_Sentence[sentence_pos++] != ','){}
	sentence_pos++; // Skip the first section as it is the $GNGGA identifier

	while(GPS_Sentence[sentence_pos] != ','){
		buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Obtain UTC time (hhmmss.ss)
	}
	buffer_pos=0;
	sentence_pos++;
	GPS_Data.hour 				= (buffer[0]-'0')*10+(buffer[1]-'0');
	GPS_Data.minute				= (buffer[2]-'0')*10+(buffer[3]-'0');
	GPS_Data.second 			= (buffer[4]-'0')*10+(buffer[5]-'0');
	GPS_Data.centisecond	= (buffer[7]-'0')*10+(buffer[8]-'0');

	while(GPS_Sentence[sentence_pos] != ','){
		buffer[buffer_pos++] = GPS_Sentence[sentence_pos++]; // Obtain packet validity
	}
	buffer_pos=0;
	sentence_pos++;
	switch(buffer[0]){
		case 'A': // Active
			break;
		case 'V':
			return;
	}
}

void GPS_Parse_Sentence(){
	/* GPS Sentence Parser */

	uint8_t sentence_pos = 0;
	uint8_t start_type = 0;
	uint8_t type[8];
	uint8_t type_pos = 0;
	while(GPS_Sentence[sentence_pos] != ','){
		if(start_type){
			type[type_pos++] = GPS_Sentence[sentence_pos];
		}
		if(GPS_Sentence[sentence_pos] == '$'){
			start_type = 1;
		}
		sentence_pos++;
	}
	start_type=0;
	if(type[0] == 'G' &&
			type[1] == 'N' &&
			type[2] == 'G' &&
			type[3] == 'G' &&
			type[4] == 'A') {
		_GPS_Parse_GGA();
	}
	else if(type[0] == 'G' &&
			type[1] == 'N' &&
			type[2] == 'R' &&
			type[3] == 'M' &&
			type[4] == 'C') {
		_GPS_Parse_RMC();
	}
}
