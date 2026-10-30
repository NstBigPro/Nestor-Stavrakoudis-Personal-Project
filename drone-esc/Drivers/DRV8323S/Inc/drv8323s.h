/**
  ******************************************************************************
  * @file           : drv8323s.h
  * @brief          : Header file for the DRV8323S gate driver
  * @author					: Nestor Stavrakoudis
  ******************************************************************************
  * Contains declarations for the SPI configuration of the DRV8323S
  ******************************************************************************
**/

#ifndef DRV8323S_H
#define DRV8323S_H

#include <stdint.h>

#define GATE_INTERFACE &hspi1	// SPI interface

enum {
	GATE_FAULT 			= 1 << 21,
	GATE_VDS_OCP 		= 1 << 20,
	GATE_GDF 				= 1 << 19,
	GATE_UVLO				= 1 << 18,
	GATE_OTSD				= 1 << 17,
	GATE_VDS_HA			= 1 << 16,
	GATE_VDS_LA			= 1 << 15,
	GATE_VDS_HB			= 1 << 14,
	GATE_VDS_LB			= 1 << 13,
	GATE_VDS_HC			= 1 << 12,
	GATE_VDS_LC			= 1 << 11,
	GATE_SA_OC			= 1 << 10,
	GATE_SB_OC			= 1 << 9,
	GATE_SC_OC			= 1 << 8,
	GATE_OTW				= 1 << 7,
	GATE_CPUV				= 1 << 6,
	GATE_VGS_HA			= 1 << 5,
	GATE_VGS_LA			= 1 << 4,
	GATE_VGS_HB			= 1 << 3,
	GATE_VGS_LB			= 1 << 2,
	GATE_VGS_HC			= 1 << 1,
	GATE_VGS_LC			= 1 << 0,
};	// Faults, taken from p. 56-57

/* Register map. Taken from p.55 */

#define GATE_FAULT_1	0x00		// Fault register 1
#define GATE_FAULT_2	0x01		// Fault register 2
#define GATE_CONTROL 0x02		// Control register
#define GATE_HS			0x03		// High-side control register
#define GATE_LS			0x04		// Low-side control register
#define GATE_OCP			0x05		// Overcurrent protection behavior register
#define GATE_CSA			0x06		// Current sense amplifier register

#define GATE_CONTROL_SETTING	0x00
#define GATE_HS_SETTING			0x344		// 120mA source/240mA sink (we drop from the defaults since 1A/2A is too aggressive for this MOSFET)
#define GATE_LS_SETTING			0x544		// Same as above. 1us TDRIVE and CBC mode. In case of a serious fault we cut off power in software.
#define GATE_OCP_SETTING			0x255		// 200ns dead time, retry mode, at 0.45V VDS level (trips at VDS level / MOSFET resistance)
#define GATE_CSA_SETTING			0x283		// Default (20V/V gain, bidirecitonal mode)

#define GATE_HS_DEFAULT			0x3FF		// Since the chip lacks a WHOAMI register we will use the HS register to check operation

#define GATE_1		0x00		// Used to distinguish the 2 gate drivers per MCU
#define GATE_2		0x01

#define GATE_READ	0x8000
#define GATE_WRITE	0x0000

#define Gate1_Enable (HAL_GPIO_WritePin(ENABLE_1_GPIO_Port, ENABLE_1_Pin, GPIO_PIN_SET))
#define Gate2_Enable (HAL_GPIO_WritePin(ENABLE_2_GPIO_Port, ENABLE_2_Pin, GPIO_PIN_SET))

#define Gate1_Disable (HAL_GPIO_WritePin(ENABLE_1_GPIO_Port, ENABLE_1_Pin, GPIO_PIN_RESET))
#define Gate2_Disable (HAL_GPIO_WritePin(ENABLE_2_GPIO_Port, ENABLE_2_Pin, GPIO_PIN_RESET))

uint16_t _Gate_ReadRegisterBlocking(uint8_t gate_driver, uint8_t reg);								// Read one register (blocking)
void 		_Gate_WriteRegisterBlocking(uint8_t gate_driver, uint8_t reg, uint16_t data);	// Write to one register (blocking)

typedef enum {
	GATE_OPERATIONAL,
	GATE_FAULTY
} GATE_StatusTypeDef;

extern volatile GATE_StatusTypeDef Gate1_Status;
extern volatile GATE_StatusTypeDef Gate2_Status;

extern uint8_t Gate_SPI_SingleBuf_RX[4];  // Receive buffer for single register (DRV8323S uses 16bit words)
extern uint8_t Gate_SPI_SingleBuf_TX[4];	// Write buffer for single register

void Gate_Test();

void _Gate1_Init();
void _Gate2_Init();
void Gate_Init();

#endif
