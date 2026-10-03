/**
  ******************************************************************************
  * @file           : bmp388.h
  * @brief          : Header file for the BMP388 barometer
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the register map and driver functions of the barometer/altimeter.
  * Taken from BMP388 datasheet, section 4.3 "Register description", p. 30-39
  ******************************************************************************
**/

#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>

#define ALT_INTERFACE &hspi3			// SPI interface


#define ALT_READ 						0x80	// MSB setting for read
#define ALT_WRITE						0x00	// MSB setting for write


#define ALT_WHO_AM_I 				0x00	// WHO_AM_I register
#define ALT_WHO_AM_I_NORMAL 	0x50	// WHO_AM_I expected response


#define ALT_ERR_REG					0x02	// Error register
#define ALT_ERR_REG_NORMAL		0x00	// Error register expected response

#define ALT_PWR_CTRL					0x1B	// Enable/disable sensor
#define ALT_PWR_SETTING			0x33	// Corresponds to temperature,pressure on, normal mode (datasheet p.36)

#define ALT_OUT_PRES_L 			0x04 	// Pressure, low byte
#define ALT_OUT_PRES_M 			0x05 	// Pressure, medium byte
#define ALT_OUT_PRES_H				0x06 	// Pressure, high byte

#define ALT_OUT_TEMP_L				0x07	// Temperature, low byte
#define ALT_OUT_TEMP_M				0x08 	// Temperature, medium byte
#define ALT_OUT_TEMP_H				0x09	// Temperature, high byte


#define ALT_OSR 							0x1C 	// Oversampling control
#define ALT_OSR_SETTING			0x03	// Corresponds to 8x pressure & 1x temperature oversampling (datasheet p.37)

#define ALT_ODR							0x1D	// Data rate control
#define ALT_ODR_SETTING			0x02	// Corresponds to 50Hz (datasheet p.38)


#define ALT_INT_CTRL					0x19	// Interrupt control register
#define ALT_INT_SETTING			0x42	// Corresponds to push pull output, fire on data ready


void 		_ALT_ReadRegister(uint8_t reg);												// Read one register (does not return, DMA is asynchronous!)
uint8_t _ALT_ReadRegisterBlocking(uint8_t reg);								// Read one register (blocking)
void 		_ALT_WriteRegisterBlocking(uint8_t reg, uint8_t data);	// Write to one register (blocking)

typedef enum {
	ALT_OPERATIONAL,
	ALT_FAULTY
} ALT_StatusTypeDef;
void ALT_Test(void);																				// Checks WHO_AM_I & error register
extern volatile ALT_StatusTypeDef ALT_Status;

void ALT_Init(void);																				// ALT initialization

typedef struct {
	uint32_t P;
	uint32_t T;
} ALT_Data_Packet;


typedef enum {
	ALT_WRITE_REG,
	ALT_SINGLE_REG,
	ALT_PACKET
} ALT_DMA_Transaction_Type;

extern ALT_DMA_Transaction_Type	ALT_Current_DMA_Transaction;	// Dictates post processing in complete callback
extern uint8_t									ALT_Current_Register_Read; 		// Transfers which register was read to complete callback

void ALT_Read(void);		// Read ALT temperature and pressure registers
extern ALT_Data_Packet ALT_Raw; // ALT output

extern uint8_t ALT_SPI_SingleBuf_RX[3]; // Receive buffer for single register (p.43,BMP388 sends dummy byte before response)
extern uint8_t ALT_SPI_SingleBuf_TX[3];	// Write buffer for single register

extern uint8_t ALT_SPI_ALTPacket_RX[14]; // Receive buffer for 6-axis measurement
extern uint8_t ALT_SPI_ALTPacket_TX[14]; // Write buffer for 6-axis measurement
#endif
