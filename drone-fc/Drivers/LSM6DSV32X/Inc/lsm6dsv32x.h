/**
  ******************************************************************************
  * @file           : lsm6dsv32x.h
  * @brief          : Header file for the LSM6DSV32X accelerometer/gyroscope
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains the register map of the IMU.
  * Taken from LSM6DSV32X datasheet, section 8 "Register mapping", p. 52-54
  ******************************************************************************
**/

#ifndef LSM6DSV32X_H
#define  LSM6DSV32X_H

#define IMU_WHO_AM_I 0x0F // WHO_AM_I register
#define IMU_WHO_AM_I_NORMAL 0x70 // WHO_AM_I expected response

#define IMU_INT1_CTRL 0x0D // Interrupt 1 control
#define IMU_INT2_CTRL 0x0E // Interrupt 2 control


#define IMU_CTRL1  0x10  // Control register 1
#define IMU_CTRL2  0x11  // Control register 2
#define IMU_CTRL3  0x12  // Control register 3
#define IMU_CTRL4  0x13  // Control register 4
#define IMU_CTRL5  0x14  // Control register 5
#define IMU_CTRL6  0x15  // Control register 6
#define IMU_CTRL7  0x16  // Control register 7
#define IMU_CTRL8  0x17  // Control register 8
#define IMU_CTRL9  0x18  // Control register 9
#define IMU_CTRL10 0x19  // Control register 10


#define IMU_OUT_TEMP_L 0x20 // Temperature, low byte
#define IMU_OUT_TEMP_H 0x21 // Temperature, high byte


#define IMU_OUTX_L_G 0x22 // Gyroscope, X axis, low byte
#define IMU_OUTX_H_G 0x23 // Gyroscope, X axis, high byte

#define IMU_OUTY_L_G 0x24 // Gyroscope, Y axis, low byte
#define IMU_OUTY_H_G 0x25 // Gyroscope, Y axis, high byte

#define IMU_OUTZ_L_G 0x26 // Gyroscope, Z axis, low byte
#define IMU_OUTZ_H_G 0x27 // Gyroscope, Z axis, high byte


#define IMU_OUTX_L_A 0x28 // Accelerometer, X axis, low byte
#define IMU_OUTX_H_A 0x29 // Accelerometer, X axis, high byte

#define IMU_OUTY_L_A 0x2A // Accelerometer, Y axis, low byte
#define IMU_OUTY_H_A 0x2B // Accelerometer, Y axis, high byte

#define IMU_OUTZ_L_A 0x2C // Accelerometer, Z axis, low byte
#define IMU_OUTZ_H_A 0x2D // Accelerometer, Z axis, high byte


#define IMU_FIFO_CTRL1 0x07 // FIFO control register 1
#define IMU_FIFO_CTRL2 0x08 // FIFO control register 2
#define IMU_FIFO_CTRL3 0x09 // FIFO control register 3
#define IMU_FIFO_CTRL4 0x0A // FIFO control register 4


#define IMU_FIFO_STATUS1 0x1B // FIFO status register 1
#define IMU_FIFO_STATUS2 0x1C // FIFO status register 2

#endif
