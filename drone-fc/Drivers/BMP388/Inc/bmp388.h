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
/* Register map & initialization settings */

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

/* Calibration coefficients */

#define ALT_PAR_T1_L					0x31
#define ALT_PAR_T1_H					0x32

#define ALT_PAR_T2_L					0x33
#define ALT_PAR_T2_H					0x34

#define ALT_PAR_T3						0x35


#define ALT_PAR_P1_L					0x36
#define ALT_PAR_P1_H					0x37

#define ALT_PAR_P2_L					0x38
#define ALT_PAR_P2_H					0x39

#define ALT_PAR_P3						0x3A

#define ALT_PAR_P4						0x3B

#define ALT_PAR_P5_L					0x3C
#define ALT_PAR_P5_H					0x3D

#define ALT_PAR_P6_L					0x3E
#define ALT_PAR_P6_H					0x3F

#define ALT_PAR_P7						0x40

#define ALT_PAR_P8						0x41

#define ALT_PAR_P9_L					0x42
#define ALT_PAR_P9_H					0x43

#define ALT_PAR_P10					0x44

#define ALT_PAR_P11					0x45




/* Driver functions & helper structs/enums */

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
} ALT_Data_Packet_Raw;

typedef struct {
	float P;
	float T;
} ALT_Data_Packet_Compensated;

typedef struct {
	uint16_t	PAR_T1;
	uint16_t	PAR_T2;
	int8_t		PAR_T3;

	int16_t		PAR_P1;
	int16_t		PAR_P2;
	int8_t		PAR_P3;
	int8_t		PAR_P4;
	uint16_t	PAR_P5;
	uint16_t	PAR_P6;
	int8_t		PAR_P7;
	int8_t		PAR_P8;
	int16_t		PAR_P9;
	int8_t		PAR_P10;
	int8_t		PAR_P11;
} ALT_Calibration_Coefficient_NVM;

typedef struct {
	float	PAR_T1;
	float	PAR_T2;
	float	PAR_T3;

	float	PAR_P1;
	float	PAR_P2;
	float	PAR_P3;
	float	PAR_P4;
	float	PAR_P5;
	float	PAR_P6;
	float	PAR_P7;
	float	PAR_P8;
	float	PAR_P9;
	float	PAR_P10;
	float	PAR_P11;
} ALT_Calibration_Coefficient_Scaled;

typedef enum {
	ALT_WRITE_REG,
	ALT_SINGLE_REG,
	ALT_PACKET
} ALT_DMA_Transaction_Type;

extern ALT_DMA_Transaction_Type	ALT_Current_DMA_Transaction;	// Dictates post processing in complete callback
extern uint8_t									ALT_Current_Register_Read; 		// Transfers which register was read to complete callback

void ALT_Read(void);		// Read ALT temperature and pressure registers
extern ALT_Data_Packet_Raw ALT_Raw; // ALT output
extern ALT_Data_Packet_Compensated ALT_Compensated;	// ALT compensated output
extern ALT_Calibration_Coefficient_NVM ALT_Coefficients_NVM;	// NVM trimming coefficients
extern ALT_Calibration_Coefficient_Scaled ALT_Coefficients_Scaled;	// Scaled trimming coefficients


void ALT_ReadCalibration(void);	// Obtain calibration/trimming data
void ALT_Compensate(void);			// Apply calibration/trimming coefficients to raw reading (formulas from datasheet appendix 9)

void _ALT_Compensate_T(void);		// Apply calibration/trimming coefficients to raw temperature reading
void _ALT_Compensate_P(void);		// Apply calibration/trimming coefficients to raw pressure reading

extern uint8_t ALT_SPI_SingleBuf_RX[3]; // Receive buffer for single register (p.43,BMP388 sends dummy byte before response)
extern uint8_t ALT_SPI_SingleBuf_TX[3];	// Write buffer for single register

extern uint8_t ALT_SPI_ALTPacket_RX[14]; // Receive buffer for pressure/temperature measurement
extern uint8_t ALT_SPI_ALTPacket_TX[14]; // Write buffer for pressure/temperature measurement

extern uint8_t ALT_SPI_ALTCoefficients_RX[23]; // Receive buffer for trimming coefficients
extern uint8_t ALT_SPI_ALTCoefficients_TX[23]; // Write buffer for trimming coefficients
#endif
