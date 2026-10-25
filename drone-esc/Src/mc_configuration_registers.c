
/*******************************************************************************
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "mc_type.h"
#include "mc_configuration_registers.h"
#include "parameters_conversion.h"

#define FIRMWARE_NAME_STR "ST MC SDK\tVer.6.4.2"

const char_t CTL_BOARD[] = "drone-esc-1";
static const char_t M1_PWR_BOARD[] = "drone-esc-1";
const char_t FIRMWARE_NAME [] = FIRMWARE_NAME_STR;

const GlobalConfig_reg_t globalConfig_reg =
{
  .SDKVersion     = SDK_VERSION,
  .MotorNumber    =  2 ,
  .MCP_Flag       = FLAG_MCP_OVER_STLINK + FLAG_MCP_OVER_UARTA + FLAG_MCP_OVER_UARTB,
  .MCPA_UARTA_LOG = 0, /* Used by Motor Pilot to detect data log enabled. */
  .MCPA_UARTB_LOG = 0,
  .MCPA_STLNK_LOG = 0,
};

static const ApplicationConfig_reg_t M1_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = 40000,
  .maxReadableCurrent = M1_MAX_READABLE_CURRENT,
  .nominalCurrent     = 20,
  .nominalVoltage     = 11,
  .driveType          = DRIVE_TYPE_M1,
};

//cstat !MISRAC2012-Rule-9.2
static const MotorConfig_reg_t M1_MotorConfig_reg =
{
  .polePairs  = 7,
  .ratedFlux  = 0.2,
  .rs         = 0.05,
  .ls         = 0.000012*1.000,
  .ld         = 0.000012,
  .maxCurrent = 20,
  .name = "FlyfishRC Flash 1804 350"
};

static const FOCFwConfig_reg_t M1_FOCConfig_reg =
{
  .primarySensor      = (uint8_t)PRIM_SENSOR_M1,
  .auxiliarySensor    = (uint8_t)AUX_SENSOR_M1,
  .topology           = (uint8_t)TOPOLOGY_M1,
  .FOCRate            = (uint8_t)REGULATION_EXECUTION_RATE,
  .PWMFrequency       = (uint32_t)PWM_FREQUENCY,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t)configurationFlag1_M1, //cstat !MISRAC2012-Rule-10.1_R6
  .configurationFlag2 = (uint16_t)configurationFlag2_M1, //cstat !MISRAC2012-Rule-10.1_R6
};

const char_t M2_PWR_BOARD[] = "drone-esc-1";

//cstat !MISRAC2012-Rule-9.2
static const MotorConfig_reg_t M2_MotorConfig_reg =
{
  .polePairs  = 7,
  .ratedFlux  = 0.2,
  .rs         = 0.05,
  .ls         = 0.000012*1.000,
  .ld         = 0.000012,
  .maxCurrent = 20,
  .name       = "FlyfishRC Flash 1804 350"
};

static const ApplicationConfig_reg_t M2_ApplicationConfig_reg =
{
  .maxMechanicalSpeed = 40000,
  .maxReadableCurrent = M2_MAX_READABLE_CURRENT,
  .nominalCurrent     = 20,
  .nominalVoltage     = 11,
  .driveType          = DRIVE_TYPE_M2,
};

static const FOCFwConfig_reg_t M2_FOCConfig_reg =
{
  .primarySensor       = (uint8_t)PRIM_SENSOR_M2,
  .auxiliarySensor    = (uint8_t)AUX_SENSOR_M2,
  .topology           = (uint8_t)TOPOLOGY_M2,
  .FOCRate            = (uint8_t)REGULATION_EXECUTION_RATE2,
  .PWMFrequency       = (uint32_t)PWM_FREQUENCY2,
  .MediumFrequency    = (uint16_t)MEDIUM_FREQUENCY_TASK_RATE2,
  .configurationFlag1 = (uint16_t)configurationFlag1_M2,
  .configurationFlag2 = (uint16_t)configurationFlag2_M2,
};

const FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS] = {&M1_FOCConfig_reg, &M2_FOCConfig_reg};
const MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS] = {&M1_MotorConfig_reg, &M2_MotorConfig_reg};
const ApplicationConfig_reg_t* ApplicationConfig_reg[NBR_OF_MOTORS] = {&M1_ApplicationConfig_reg,
                                                                       &M2_ApplicationConfig_reg};
const char_t * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD ,M2_PWR_BOARD};

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
