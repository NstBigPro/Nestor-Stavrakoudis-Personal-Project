/**
  ******************************************************************************
  * @file           : bmp388.c
  * @brief          : Driver for the BMP388 barometer
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the driver functions of the barometer/altimeter.
  ******************************************************************************
**/

#include "bmp388.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* extern variable initialization */
volatile ALT_StatusTypeDef ALT_Status = ALT_FAULTY;
ALT_DMA_Transaction_Type	ALT_Current_DMA_Transaction;
uint8_t         ALT_Current_Register_Read;
ALT_Data_Packet_Raw ALT_Raw;
ALT_Data_Packet_Compensated ALT_Compensated;
ALT_Calibration_Coefficient_NVM ALT_Coefficients_NVM;
ALT_Calibration_Coefficient_Scaled ALT_Coefficients_Scaled;
uint8_t ALT_SPI_SingleBuf_RX[3], ALT_SPI_SingleBuf_TX[3];
uint8_t ALT_SPI_ALTPacket_RX[14], ALT_SPI_ALTPacket_TX[14];
uint8_t ALT_SPI_ALTCoefficients_RX[23], ALT_SPI_ALTCoefficients_TX[23];


void _ALT_ReadRegister(uint8_t reg){
	/* Read singular register */

	ALT_SPI_SingleBuf_TX[0] = reg | ALT_READ;
	ALT_SPI_SingleBuf_TX[1] = 0x00;

	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	ALT_Current_DMA_Transaction = ALT_SINGLE_REG;
	ALT_Current_Register_Read		= reg;

	HAL_SPI_TransmitReceive_DMA(ALT_INTERFACE,ALT_SPI_SingleBuf_TX,ALT_SPI_SingleBuf_RX,3);
}

uint8_t _ALT_ReadRegisterBlocking(uint8_t reg){
	/* Read singular register (blocking)*/

	ALT_SPI_SingleBuf_TX[0] = reg | ALT_READ;
	ALT_SPI_SingleBuf_TX[1] = 0x00;

	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	ALT_Current_DMA_Transaction = ALT_SINGLE_REG;
	ALT_Current_Register_Read		= reg;
	if (HAL_SPI_TransmitReceive(ALT_INTERFACE, ALT_SPI_SingleBuf_TX, ALT_SPI_SingleBuf_RX, 3, 10) != HAL_OK) {
			ALT_Status = ALT_FAULTY;
	}
	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_SET);
	return ALT_SPI_SingleBuf_RX[2];
}

void _ALT_WriteRegisterBlocking(uint8_t reg, uint8_t data){
	/* Write to singular register */

	ALT_SPI_SingleBuf_TX[0] = reg | ALT_WRITE;
	ALT_SPI_SingleBuf_TX[1] = data;

	ALT_SPI_SingleBuf_RX[0] = 0x00;
	ALT_SPI_SingleBuf_RX[1] = 0x00;


	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	ALT_Current_DMA_Transaction = ALT_WRITE_REG;

	if (HAL_SPI_TransmitReceive(ALT_INTERFACE, ALT_SPI_SingleBuf_TX, ALT_SPI_SingleBuf_RX, 2, 10) != HAL_OK) {
	    ALT_Status = ALT_FAULTY;
	}
	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_SET);
}

void ALT_Test(){
	/* Check WHO_AM_I register */
	switch(_ALT_ReadRegisterBlocking(ALT_WHO_AM_I)) {
		case ALT_WHO_AM_I_NORMAL:
			ALT_Status = ALT_OPERATIONAL;
			break;
		default:
			ALT_Status = ALT_FAULTY;
			break;
	}
	switch(_ALT_ReadRegisterBlocking(ALT_ERR_REG) & 0x01) { // Last bit of the register is the fatal error
		case 0:
			break;
		default:
			ALT_Status = ALT_FAULTY;
			break;
	}
}

void ALT_ReadCalibration(){
	/* Obtain calibration/trimming data */

	ALT_SPI_ALTCoefficients_TX[0] = ALT_PAR_T1_L | ALT_READ;

	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive(ALT_INTERFACE, ALT_SPI_ALTCoefficients_TX, ALT_SPI_ALTCoefficients_RX, 23, 10) != HAL_OK) {
			ALT_Status = ALT_FAULTY;
	}
	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_SET);

	ALT_Coefficients_NVM.PAR_T1		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_T1_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_T1_L-0x2F];
	ALT_Coefficients_NVM.PAR_T2		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_T2_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_T2_L-0x2F];
	ALT_Coefficients_NVM.PAR_T3		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_T3-0x2F]);

	ALT_Coefficients_NVM.PAR_P1		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P1_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_P1_L-0x2F];
	ALT_Coefficients_NVM.PAR_P2		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P2_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_P2_L-0x2F];
	ALT_Coefficients_NVM.PAR_P3		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P3-0x2F]);
	ALT_Coefficients_NVM.PAR_P4		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P4-0x2F]);
	ALT_Coefficients_NVM.PAR_P5		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P5_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_P5_L-0x2F];
	ALT_Coefficients_NVM.PAR_P6		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P6_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_P6_L-0x2F];
	ALT_Coefficients_NVM.PAR_P7		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P7-0x2F]);
	ALT_Coefficients_NVM.PAR_P8		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P8-0x2F]);
	ALT_Coefficients_NVM.PAR_P9		= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P9_H-0x2F] << 8)
															+  ALT_SPI_ALTCoefficients_RX[ALT_PAR_P9_L-0x2F];
	ALT_Coefficients_NVM.PAR_P10	= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P10-0x2F]);
	ALT_Coefficients_NVM.PAR_P11	= (ALT_SPI_ALTCoefficients_RX[ALT_PAR_P11-0x2F]);

	ALT_Coefficients_Scaled.PAR_T1 = ALT_Coefficients_NVM.PAR_T1 * 0x1p8f;			// Read datasheet appendix 9
	ALT_Coefficients_Scaled.PAR_T2 = ALT_Coefficients_NVM.PAR_T2 * 0x1p-30f;
	ALT_Coefficients_Scaled.PAR_T3 = ALT_Coefficients_NVM.PAR_T3 * 0x1p-48f;

	ALT_Coefficients_Scaled.PAR_P1 = (ALT_Coefficients_NVM.PAR_P1 - 0x1p14f) * 0x1p-20f;
	ALT_Coefficients_Scaled.PAR_P2 = (ALT_Coefficients_NVM.PAR_P2 - 0x1p14f) * 0x1p-29f;
	ALT_Coefficients_Scaled.PAR_P3 = ALT_Coefficients_NVM.PAR_P3 * 0x1p-32f;
	ALT_Coefficients_Scaled.PAR_P4 = ALT_Coefficients_NVM.PAR_P4 * 0x1p-37f;
	ALT_Coefficients_Scaled.PAR_P5 = ALT_Coefficients_NVM.PAR_P5 * 0x1p3f;
	ALT_Coefficients_Scaled.PAR_P6 = ALT_Coefficients_NVM.PAR_P6 * 0x1p-6f;
	ALT_Coefficients_Scaled.PAR_P7 = ALT_Coefficients_NVM.PAR_P7 * 0x1p-8f;
	ALT_Coefficients_Scaled.PAR_P8 = ALT_Coefficients_NVM.PAR_P8 * 0x1p-15f;
	ALT_Coefficients_Scaled.PAR_P9 = ALT_Coefficients_NVM.PAR_P9 * 0x1p-48f;
	ALT_Coefficients_Scaled.PAR_P10 = ALT_Coefficients_NVM.PAR_P10 * 0x1p-48f;
	ALT_Coefficients_Scaled.PAR_P11 = ALT_Coefficients_NVM.PAR_P11 * 0x1p-65f;




}

void ALT_Init(){
	/* ALT initialization settings */
	_ALT_WriteRegisterBlocking(ALT_ODR, ALT_ODR_SETTING); // Set barometer ODR

	_ALT_WriteRegisterBlocking(ALT_OSR, ALT_OSR_SETTING); // Set barometer OSR

	_ALT_WriteRegisterBlocking(ALT_INT_CTRL, ALT_INT_SETTING); // Set interrupt behavior

	_ALT_WriteRegisterBlocking(ALT_PWR_CTRL, ALT_PWR_SETTING); // Power on the sensor

	ALT_ReadCalibration();
}

void ALT_Read(){
	/* Obtain 1 measurement packet */

	ALT_SPI_ALTPacket_TX[0] = ALT_OUT_PRES_L | ALT_READ;

	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	ALT_Current_DMA_Transaction = ALT_PACKET;
	HAL_SPI_TransmitReceive_DMA(ALT_INTERFACE,ALT_SPI_ALTPacket_TX,ALT_SPI_ALTPacket_RX,14);
}

void _ALT_Compensate_T(){
	/* Apply calibration/trimming coefficients to raw temperature reading */

	float ALT_Partial_T1;
	float ALT_Partial_T2;

	ALT_Partial_T1 = (float)(ALT_Raw.T - ALT_Coefficients_Scaled.PAR_T1);
	ALT_Partial_T2 = (float)(ALT_Partial_T1*ALT_Coefficients_Scaled.PAR_T2);

	ALT_Compensated.T = ALT_Partial_T2 + (ALT_Partial_T1 * ALT_Partial_T1) * ALT_Coefficients_Scaled.PAR_T3;
}

void _ALT_Compensate_P(){
	/* Apply calibration/trimming coefficients to raw pressure reading */

	float ALT_Partial_P1;
	float ALT_Partial_P2;
	float ALT_Partial_P3;
	float ALT_Partial_P4;

	float ALT_Out_P1;
	float ALT_Out_P2;

	ALT_Partial_P1 = ALT_Coefficients_Scaled.PAR_P6 * ALT_Compensated.T;
	ALT_Partial_P2 = ALT_Coefficients_Scaled.PAR_P7 * (ALT_Compensated.T * ALT_Compensated.T);
	ALT_Partial_P3 = ALT_Coefficients_Scaled.PAR_P8 * (ALT_Compensated.T * ALT_Compensated.T * ALT_Compensated.T);
	ALT_Out_P1 = ALT_Coefficients_Scaled.PAR_P5 + ALT_Partial_P1 + ALT_Partial_P2 + ALT_Partial_P3;

	ALT_Partial_P1 = ALT_Coefficients_Scaled.PAR_P2 * ALT_Compensated.T;
	ALT_Partial_P2 = ALT_Coefficients_Scaled.PAR_P3 * (ALT_Compensated.T * ALT_Compensated.T);
	ALT_Partial_P3 = ALT_Coefficients_Scaled.PAR_P4 * (ALT_Compensated.T * ALT_Compensated.T * ALT_Compensated.T);
	ALT_Out_P2 = (float) ALT_Raw.P *
				(ALT_Coefficients_Scaled.PAR_P1 + ALT_Partial_P1 + ALT_Partial_P2 + ALT_Partial_P3);

	ALT_Partial_P1 = (float) ALT_Raw.P * (float) ALT_Raw.P;
	ALT_Partial_P2 = ALT_Coefficients_Scaled.PAR_P9 + ALT_Coefficients_Scaled.PAR_P10 * ALT_Compensated.T;
	ALT_Partial_P3 = ALT_Partial_P1 * ALT_Partial_P2;
	ALT_Partial_P4 = ALT_Partial_P3 + ((float)ALT_Raw.P *(float)ALT_Raw.P *(float)ALT_Raw.P)
								* ALT_Coefficients_Scaled.PAR_P11;

	ALT_Compensated.P = ALT_Out_P1 + ALT_Out_P2 + ALT_Partial_P4;
}

void ALT_Compensate(){
	/* Apply calibration/trimming coefficients to raw reading */

	_ALT_Compensate_T();
	_ALT_Compensate_P();

}
