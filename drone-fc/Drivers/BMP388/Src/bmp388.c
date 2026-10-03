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
ALT_Data_Packet ALT_Raw;
uint8_t ALT_SPI_SingleBuf_RX[3], ALT_SPI_SingleBuf_TX[3];
uint8_t ALT_SPI_ALTPacket_RX[14], ALT_SPI_ALTPacket_TX[14];


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

void ALT_Init(){
	/* ALT initialization settings */
	_ALT_WriteRegisterBlocking(ALT_ODR, ALT_ODR_SETTING); // Set barometer ODR

	_ALT_WriteRegisterBlocking(ALT_OSR, ALT_OSR_SETTING); // Set barometer OSR

	_ALT_WriteRegisterBlocking(ALT_INT_CTRL, ALT_INT_SETTING); // Set interrupt behavior

	_ALT_WriteRegisterBlocking(ALT_PWR_CTRL, ALT_PWR_SETTING); // Power on the sensor

}

void ALT_Read(){
	/* Obtain 1 measurement packet */

	ALT_SPI_ALTPacket_TX[0] = ALT_OUT_PRES_L | ALT_READ;

	HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

	ALT_Current_DMA_Transaction = ALT_PACKET;
	HAL_SPI_TransmitReceive_DMA(ALT_INTERFACE,ALT_SPI_ALTPacket_TX,ALT_SPI_ALTPacket_RX,14);
}
