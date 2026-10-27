/**
  ******************************************************************************
  * @file           : ep1.c
  * @brief          : Header file for the LSM6DSV32X accelerometer/gyroscope
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains driver functions for the CRSF ELRS parser for the Happymodel EP1 receiver.
  ******************************************************************************
**/

#include "ep1.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <math.h>


static uint16_t Radio_Read_Pos = 0;
static uint16_t Radio_Write_Pos = 0;
static uint16_t Radio_Sentence_Pos = 0;
static uint16_t Radio_In_Sentence = 0;
static uint8_t	Radio_Sentence_Length = 0;

/* Extern variable declarations */

volatile Radio_Packet Radio_Data;
volatile Radio_Packet_Scaled Radio_Scaled;					// Scaled radio data packet
uint8_t Radio_Buffer_RX[512];

volatile uint8_t Radio_Sentence[512];

volatile uint32_t Radio_Timestamp = 0;


void Radio_Init(){
	/* Radio initialization */
	HAL_UART_Receive_DMA(RADIO_INTERFACE,Radio_Buffer_RX,512);
	__HAL_UART_ENABLE_IT(RADIO_INTERFACE,UART_IT_IDLE);		// Enable the UART idle interrupt
}

void Radio_New_Sentence(){
	/* Callback for new sentence */
	Radio_Write_Pos = 512 - __HAL_DMA_GET_COUNTER(RADIO_HDMARX); // Check where DMA finished

	while(Radio_Read_Pos != Radio_Write_Pos){
		uint8_t Radio_Character = Radio_Buffer_RX[Radio_Read_Pos++];

		Radio_Parse_Byte(Radio_Character);

		if(Radio_Read_Pos >= 512){
			Radio_Read_Pos = 0;
		}
	}
}

void _Radio_Parse_RCChannelsPacked(void){
	/* RC Channels Packed sentence parser */


	uint8_t crc_calculated = 0;
  for (int sentence_pos = 2; sentence_pos < Radio_Sentence_Length-1; sentence_pos++) { // Calculate the crc_calculated8 checksum
      crc_calculated ^= Radio_Sentence[sentence_pos];
      for (int i = 0; i < 8; i++)
          crc_calculated = (crc_calculated & 0x80) ? (crc_calculated << 1) ^ 0xD5 : (crc_calculated << 1);
  }
  uint8_t crc_provided = Radio_Sentence[Radio_Sentence_Length-1];
  if(crc_provided == crc_calculated){
  	Radio_Timestamp = HAL_GetTick();

  	Radio_Data.checksum_validity = CRC8_VALID;
		/* We follow the AETR convention (first roll, then pitch, then throttle, then yaw */

		Radio_Data.roll = ((uint16_t)Radio_Sentence[3]       | (uint16_t)Radio_Sentence[4] << 8)                    								     			& 0x07FF;
		Radio_Data.pitch = ((uint16_t)Radio_Sentence[4] >> 3  | (uint16_t)Radio_Sentence[5] << 5)                    								    			& 0x07FF;
		Radio_Data.throttle = ((uint16_t)Radio_Sentence[5] >> 6  | (uint16_t)Radio_Sentence[6] << 2 | (uint16_t)Radio_Sentence[7] << 10) 			& 0x07FF;
		Radio_Data.yaw = ((uint16_t)Radio_Sentence[7] >> 1  | (uint16_t)Radio_Sentence[8] << 7)                     									    		& 0x07FF;

		/* The rest of the channels are auxiliary, we arbitrarily set the next channels to be the armed and kill signals */
		Radio_Data.armed =(uint16_t) ((uint16_t)Radio_Sentence[8] >> 4  | (uint16_t)Radio_Sentence[9] << 4)                       								& 0x07FF;
		Radio_Data.kill = (uint16_t) ((uint16_t)Radio_Sentence[9] >> 7  | (uint16_t)Radio_Sentence[10] << 1 | (uint16_t)Radio_Sentence[11] << 9)	& 0x07FF;

		Radio_Scaled.roll = (Radio_Data.roll - RADIO_CENTER) * ROLL_MAX / (float) ((RADIO_MAX-RADIO_MIN)/2.0f);
		Radio_Scaled.pitch = (Radio_Data.pitch - RADIO_CENTER) * PITCH_MAX / (float) ((RADIO_MAX-RADIO_MIN)/2.0f);
		Radio_Scaled.yaw = (Radio_Data.yaw - RADIO_CENTER) * YAW_MAX / (float) ((RADIO_MAX-RADIO_MIN)/2.0f);

		Radio_Scaled.throttle = (Radio_Data.throttle - RADIO_MIN) / (float) (RADIO_MAX - RADIO_MIN);

		if(Radio_Scaled.roll>ROLL_MAX)
			Radio_Scaled.roll = ROLL_MAX;
		else if(Radio_Scaled.roll<-ROLL_MAX)
			Radio_Scaled.roll = -ROLL_MAX;

		if(Radio_Scaled.pitch>PITCH_MAX)
			Radio_Scaled.pitch = PITCH_MAX;
		else if(Radio_Scaled.pitch<-PITCH_MAX)
			Radio_Scaled.pitch = -PITCH_MAX;

		if(Radio_Scaled.yaw>YAW_MAX)
			Radio_Scaled.yaw = YAW_MAX;
		else if(Radio_Scaled.yaw<-YAW_MAX)
			Radio_Scaled.yaw = -YAW_MAX;

		if(Radio_Scaled.throttle>1)
			Radio_Scaled.throttle = 1;
		else if(Radio_Scaled.throttle<0)
			Radio_Scaled.throttle = 0;

		Radio_Scaled.armed_position = Radio_Data.armed;

		if(Radio_Data.kill>RADIO_CENTER)
			Radio_Scaled.kill = 1;
		else
			Radio_Scaled.kill = 0;

  }
  else
  	Radio_Data.checksum_validity = CRC8_INVALID;

}

void Radio_Parse_Sentence(){
	/* Sentence parser */
	if(Radio_Sentence[1] + 2 == RADIO_RC_CHANNELS_PACKED_LENGTH &&
			Radio_Sentence[2] == RADIO_RC_CHANNELS_PACKED_BYTE)
		_Radio_Parse_RCChannelsPacked();
}

void Radio_Parse_Byte(uint8_t byte){
	/* Byte parser */
	if(byte == RADIO_SYNC_BYTE && !Radio_In_Sentence){
		Radio_Sentence_Pos = 0;
		Radio_In_Sentence = 1;
	}
	if(!Radio_In_Sentence){
		return;
	}

	Radio_Sentence[Radio_Sentence_Pos] = byte;
	if(Radio_Sentence_Pos == 1){	// 2nd byte is how many bytes follow the 2 bytes of sync+length. (0x18 for RC Channels Packed)
		if(byte >= 2 && byte <= 62)
			Radio_Sentence_Length = byte + 2;
		else
			return;
	}
	Radio_Sentence_Pos++;
	if(Radio_Sentence_Pos == Radio_Sentence_Length){
		Radio_Parse_Sentence();
		Radio_In_Sentence = 0;
	}
}


