/**
  ******************************************************************************
  * @file           : lsm6dsv32x.c
  * @brief          : Driver for the LSM6DSV32X accelerometer/gyroscope
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the driver functions of the IMU.
  ******************************************************************************
**/

#include "lsm6dsv32x.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* extern variable initialization */
volatile IMU_StatusTypeDef IMU_Status = IMU_FAULTY;
IMU_DMA_Transaction_Type	IMU_Current_DMA_Transaction;
uint8_t         IMU_Current_Register_Read;
IMU_Data_Packet IMU_Raw;
uint8_t IMU_SPI_SingleBuf_RX[2], IMU_SPI_SingleBuf_TX[2];
uint8_t IMU_SPI_IMUPacket_RX[13], IMU_SPI_IMUPacket_TX[13];


void _IMU_ReadRegister(uint8_t reg){
	/* Read singular register */

	IMU_SPI_SingleBuf_TX[0] = reg | IMU_READ;
	IMU_SPI_SingleBuf_TX[1] = 0x00;

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	IMU_Current_DMA_Transaction = IMU_SINGLE_REG;
	IMU_Current_Register_Read		= reg;

	HAL_SPI_TransmitReceive_DMA(IMU_INTERFACE,IMU_SPI_SingleBuf_TX,IMU_SPI_SingleBuf_RX,2);
}

uint8_t _IMU_ReadRegisterBlocking(uint8_t reg){
	/* Read singular register (blocking)*/

	IMU_SPI_SingleBuf_TX[0] = reg | IMU_READ;
	IMU_SPI_SingleBuf_TX[1] = 0x00;

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	IMU_Current_DMA_Transaction = IMU_SINGLE_REG;
	IMU_Current_Register_Read		= reg;

	if (HAL_SPI_TransmitReceive(IMU_INTERFACE, IMU_SPI_SingleBuf_TX, IMU_SPI_SingleBuf_RX, 2, 10) != HAL_OK) {
			IMU_Status = IMU_FAULTY;
	}
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
	return IMU_SPI_SingleBuf_RX[1];
}

void _IMU_WriteRegisterBlocking(uint8_t reg, uint8_t data){
	/* Write to singular register */

	IMU_SPI_SingleBuf_TX[0] = reg | IMU_WRITE;
	IMU_SPI_SingleBuf_TX[1] = data;

	IMU_SPI_SingleBuf_RX[0] = 0x00;
	IMU_SPI_SingleBuf_RX[1] = 0x00;


	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	IMU_Current_DMA_Transaction = IMU_WRITE_REG;

	if (HAL_SPI_TransmitReceive(IMU_INTERFACE, IMU_SPI_SingleBuf_TX, IMU_SPI_SingleBuf_RX, 2, 10) != HAL_OK) {
	    IMU_Status = IMU_FAULTY;
	}
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void IMU_Test(){
	/* Check WHO_AM_I register */
	switch(_IMU_ReadRegisterBlocking(IMU_WHO_AM_I)) {
		case IMU_WHO_AM_I_NORMAL:
			IMU_Status = IMU_OPERATIONAL;
			break;
		default:
			IMU_Status = IMU_FAULTY;
			break;
	}
}

void IMU_Init(){
	/* IMU initialization settings */
	_IMU_WriteRegisterBlocking(IMU_CTRL1, IMU_ACCEL_ODR); // Set accelerometer ODR
	_IMU_WriteRegisterBlocking(IMU_CTRL2, IMU_GYRO_ODR); 	// Set gyroscope ODR


	_IMU_WriteRegisterBlocking(IMU_CTRL8, IMU_ACCEL_SCALE); // Set accelerometer scale
	_IMU_WriteRegisterBlocking(IMU_CTRL6, IMU_GYRO_SCALE);  // Set gyroscope scale

	_IMU_WriteRegisterBlocking(IMU_CTRL4, IMU_CTRL4_SETTING);	// Set INT1 to pulsed instead of latching mode

	_IMU_WriteRegisterBlocking(IMU_INT1_CTRL, IMU_INT1_SETTING); // Set INT1 behavior

}

void IMU_Read(){
	/* Obtain 1 measurement packet */

	IMU_SPI_IMUPacket_TX[0] = IMU_OUTX_L_G | IMU_READ;

	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

	IMU_Current_DMA_Transaction = IMU_PACKET;
	HAL_SPI_TransmitReceive_DMA(IMU_INTERFACE,IMU_SPI_IMUPacket_TX,IMU_SPI_IMUPacket_RX,13);
}

