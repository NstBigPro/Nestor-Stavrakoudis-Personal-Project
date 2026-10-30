/**
  ******************************************************************************
  * @file           : drv8323s.c
  * @brief          : Driver for the DRV8323S gate driver
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the driver functions of the DRV8323S gate driver.
  ******************************************************************************
**/

#include "drv8323s.h"

#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* extern variable initialization */
volatile GATE_StatusTypeDef Gate1_Status = GATE_FAULTY;
volatile GATE_StatusTypeDef Gate2_Status = GATE_FAULTY;

uint8_t Gate_SPI_SingleBuf_RX[4];  // Receive buffer for single register (DRV8323S uses 16bit words)
uint8_t Gate_SPI_SingleBuf_TX[4];	// Write buffer for single register

uint16_t _Gate_ReadRegisterBlocking(uint8_t gate_driver, uint8_t reg){
	/* Read singular register (blocking)*/
	uint16_t frame = GATE_READ | (reg << 11);
	Gate_SPI_SingleBuf_TX[0] = frame & 0xFF;
	Gate_SPI_SingleBuf_TX[1] = frame >> 8;
	Gate_SPI_SingleBuf_TX[2] = 0x00;
	Gate_SPI_SingleBuf_TX[3] = 0x00;

	if(gate_driver == GATE_1)
		HAL_GPIO_WritePin(NSCS_1_GPIO_Port, NSCS_1_Pin, GPIO_PIN_RESET);
	else if(gate_driver == GATE_2)
		HAL_GPIO_WritePin(NSCS_2_GPIO_Port, NSCS_2_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive(GATE_INTERFACE, Gate_SPI_SingleBuf_TX, Gate_SPI_SingleBuf_RX, 1, 10) != HAL_OK) {
			if(gate_driver == GATE_1) Gate1_Status = GATE_FAULTY;
			if(gate_driver == GATE_2) Gate2_Status = GATE_FAULTY;
	}
	if(gate_driver == GATE_1)
		HAL_GPIO_WritePin(NSCS_1_GPIO_Port, NSCS_1_Pin, GPIO_PIN_SET);
	else if(gate_driver == GATE_2)
		HAL_GPIO_WritePin(NSCS_2_GPIO_Port, NSCS_2_Pin, GPIO_PIN_SET);
	return ((Gate_SPI_SingleBuf_RX[1] << 8) + Gate_SPI_SingleBuf_RX[0]) & 0x7FF;
}
void _Gate_WriteRegisterBlocking(uint8_t gate_driver, uint8_t reg, uint16_t data){
	/* Write to singular register */
	uint16_t frame = GATE_WRITE | (reg << 11) | (data & 0x7FF);
	Gate_SPI_SingleBuf_TX[0] = frame & 0xFF;
	Gate_SPI_SingleBuf_TX[1] = frame >> 8;
	Gate_SPI_SingleBuf_TX[2] = 0x00;
	Gate_SPI_SingleBuf_TX[3] = 0x00;

	Gate_SPI_SingleBuf_RX[0] = 0x00;
	Gate_SPI_SingleBuf_RX[1] = 0x00;
	Gate_SPI_SingleBuf_RX[2] = 0x00;
	Gate_SPI_SingleBuf_RX[3] = 0x00;


	if(gate_driver == GATE_1)
		HAL_GPIO_WritePin(NSCS_1_GPIO_Port, NSCS_1_Pin, GPIO_PIN_RESET);
	else if(gate_driver == GATE_2)
		HAL_GPIO_WritePin(NSCS_2_GPIO_Port, NSCS_2_Pin, GPIO_PIN_RESET);

	if (HAL_SPI_TransmitReceive(GATE_INTERFACE, Gate_SPI_SingleBuf_TX, Gate_SPI_SingleBuf_RX, 1, 10) != HAL_OK) {
			if(gate_driver == GATE_1) Gate1_Status = GATE_FAULTY;
			if(gate_driver == GATE_2) Gate2_Status = GATE_FAULTY;
	}
	if(gate_driver == GATE_1)
		HAL_GPIO_WritePin(NSCS_1_GPIO_Port, NSCS_1_Pin, GPIO_PIN_SET);
	else if(gate_driver == GATE_2)
		HAL_GPIO_WritePin(NSCS_2_GPIO_Port, NSCS_2_Pin, GPIO_PIN_SET);
}

void Gate_Test(){
	/* Check the HS register's default value */
	switch(_Gate_ReadRegisterBlocking(GATE_1,GATE_HS)) {
		case GATE_HS_DEFAULT:
			Gate1_Status = GATE_OPERATIONAL;
			break;
		default:
			Gate1_Status = GATE_FAULTY;
			break;
	}
	switch(_Gate_ReadRegisterBlocking(GATE_2,GATE_HS)) {
		case GATE_HS_DEFAULT:
			Gate2_Status = GATE_OPERATIONAL;
			break;
		default:
			Gate2_Status = GATE_FAULTY;
			break;
	}
}
void _Gate1_Init(){
	_Gate_WriteRegisterBlocking(GATE_1,GATE_CONTROL,GATE_CONTROL_SETTING);
	_Gate_WriteRegisterBlocking(GATE_1,GATE_HS,GATE_HS_SETTING);
	_Gate_WriteRegisterBlocking(GATE_1,GATE_LS,GATE_LS_SETTING);
	_Gate_WriteRegisterBlocking(GATE_1,GATE_OCP,GATE_OCP_SETTING);
	_Gate_WriteRegisterBlocking(GATE_1,GATE_CSA,GATE_CSA_SETTING);
}

void _Gate2_Init(){
	_Gate_WriteRegisterBlocking(GATE_2,GATE_CONTROL,GATE_CONTROL_SETTING);
	_Gate_WriteRegisterBlocking(GATE_2,GATE_HS,GATE_HS_SETTING);
	_Gate_WriteRegisterBlocking(GATE_2,GATE_LS,GATE_LS_SETTING);
	_Gate_WriteRegisterBlocking(GATE_2,GATE_OCP,GATE_OCP_SETTING);
	_Gate_WriteRegisterBlocking(GATE_2,GATE_CSA,GATE_CSA_SETTING);
}
void Gate_Init(){
	_Gate1_Init();
	_Gate2_Init();
}
