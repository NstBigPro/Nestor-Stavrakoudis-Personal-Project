/**
  ******************************************************************************
  * @file           : lsm6dsv32x.h
  * @brief          : Header file for the LSM6DSV32X accelerometer/gyroscope
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the register map and driver functions of the IMU.
  * Taken from LSM6DSV32X datasheet, section 8 "Register mapping", p. 52-54
  ******************************************************************************
**/

/* Register map & initialization settings */

#ifndef LSM6DSV32X_H
#define  LSM6DSV32X_H

#include <stdint.h>

#define IMU_INTERFACE &hspi1			// SPI interface


#define IMU_READ 						0x80	// MSB setting for read
#define IMU_WRITE						0x00	// MSB setting for write

#define IMU_WHO_AM_I				 	0x0F 	// WHO_AM_I register
#define IMU_WHO_AM_I_NORMAL 	0x70 	// WHO_AM_I expected response

#define IMU_INT1_CTRL				0x0D	// Interrupt 1 control
#define IMU_INT2_CTRL 				0x0E 	// Interrupt 2 control

#define IMU_CTRL1  					0x10  // Control register 1
#define IMU_CTRL2  					0x11  // Control register 2
#define IMU_CTRL3  					0x12  // Control register 3
#define IMU_CTRL4 						0x13  // Control register 4
#define IMU_CTRL5  					0x14  // Control register 5
#define IMU_CTRL6  					0x15  // Control register 6
#define IMU_CTRL7  					0x16  // Control register 7
#define IMU_CTRL8  					0x17  // Control register 8
#define IMU_CTRL9  					0x18  // Control register 9
#define IMU_CTRL10 					0x19  // Control register 10


#define IMU_OUT_TEMP_L 			0x20 	// Temperature, low byte
#define IMU_OUT_TEMP_H				0x21 	// Temperature, high byte


#define IMU_OUTX_L_G 				0x22 	// Gyroscope, X axis, low byte
#define IMU_OUTX_H_G 				0x23 	// Gyroscope, X axis, high byte

#define IMU_OUTY_L_G 				0x24	// Gyroscope, Y axis, low byte
#define IMU_OUTY_H_G 				0x25 	// Gyroscope, Y axis, high byte

#define IMU_OUTZ_L_G 				0x26 	// Gyroscope, Z axis, low byte
#define IMU_OUTZ_H_G 				0x27 	// Gyroscope, Z axis, high byte


#define IMU_OUTX_L_A 				0x28 	// Accelerometer, X axis, low byte
#define IMU_OUTX_H_A 				0x29	// Accelerometer, X axis, high byte

#define IMU_OUTY_L_A					0x2A 	// Accelerometer, Y axis, low byte
#define IMU_OUTY_H_A 				0x2B	// Accelerometer, Y axis, high byte

#define IMU_OUTZ_L_A 				0x2C 	// Accelerometer, Z axis, low byte
#define IMU_OUTZ_H_A 				0x2D 	// Accelerometer, Z axis, high byte


#define IMU_FIFO_CTRL1 			0x07 	// FIFO control register 1
#define IMU_FIFO_CTRL2 			0x08 	// FIFO control register 2
#define IMU_FIFO_CTRL3				0x09 	// FIFO control register 3
#define IMU_FIFO_CTRL4 			0x0A 	// FIFO control register 4


#define IMU_FIFO_STATUS1 		0x1B 	// FIFO status register 1
#define IMU_FIFO_STATUS2 		0x1C	// FIFO status register 2

#define IMU_ACCEL_ODR				0x0A	// This setting corresponds to High-Performance mode, 1.92kHz ODR (datasheet p.65)
#define IMU_GYRO_ODR					0x0A	// This setting corresponds to High-Performance mode, 1.92kHz ODR (datasheet p.66)

#define IMU_INT1_SETTING 		0x02 	// This setting fires INT1 on gyroscope data ready (datasheet p.63)

#define IMU_CTRL4_SETTING		0x02	// This setting sets INT1 to pulsed instead of latched mode

#define IMU_ACCEL_SCALE 			0x07	// This setting corresponds to +-32g  (datasheet p.71)
#define IMU_GYRO_SCALE  			0x04	// This setting corresponds to +-2000dps (datasheet p.69-70)


void 		_IMU_ReadRegister(uint8_t reg);												// Read one register (does not return, DMA is asynchronous!)
uint8_t _IMU_ReadRegisterBlocking(uint8_t reg);								// Read one register (blocking)
void 		_IMU_WriteRegisterBlocking(uint8_t reg, uint8_t data);// Write to one register (blocking)



typedef enum {
	IMU_OPERATIONAL,
	IMU_FAULTY
} IMU_StatusTypeDef;
void IMU_Test(void);																				// Checks WHO_AM_I
extern volatile IMU_StatusTypeDef IMU_Status;

void IMU_Init(void);																				// IMU initialization

typedef struct {
	int16_t A_X;
	int16_t A_Y;
	int16_t A_Z;
	int16_t G_X;
	int16_t G_Y;
	int16_t G_Z;
} IMU_Data_Packet;

typedef enum {
	IMU_WRITE_REG,
	IMU_SINGLE_REG,
	IMU_PACKET
} IMU_DMA_Transaction_Type;

extern IMU_DMA_Transaction_Type	IMU_Current_DMA_Transaction;	// Dictates post processing in complete callback
extern uint8_t									IMU_Current_Register_Read; 		// Transfers which register was read to complete callback

void IMU_Read(void);		// Read IMU acceleration and gyroscope registers
extern IMU_Data_Packet IMU_Raw; // IMU output

extern uint8_t IMU_SPI_SingleBuf_RX[2]; // Receive buffer for single register
extern uint8_t IMU_SPI_SingleBuf_TX[2];	// Write buffer for single register

extern uint8_t IMU_SPI_IMUPacket_RX[13]; // Receive buffer for 6-axis measurement
extern uint8_t IMU_SPI_IMUPacket_TX[13]; // Write buffer for 6-axis measurement

#endif
