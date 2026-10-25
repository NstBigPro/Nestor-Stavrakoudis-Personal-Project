
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "pqd_motor_power_measurement.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .ConvFact = PQD_CONVERSION_FACTOR
};

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2=
{
  .ConvFact = PQD_CONVERSION_FACTOR2
};

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(IQMAX * SP_KIDIV),
  .wLowerIntegralLimit = -(int32_t)(IQMAX * SP_KIDIV),
  .hUpperOutputLimit   = (int16_t)IQMAX,
  .hLowerOutputLimit   = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1.
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 1.
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV,
  .hKiDivisor          = (uint16_t)TF_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1.
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveTorque          = (int16_t)NOMINAL_CURRENT,
  .MinNegativeTorque          = -(int16_t)NOMINAL_CURRENT,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .TorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT,
};

RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,
  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED_UNIT/2),
  .OTFStartupEnabled       = false,

  .OTFPhaseParams =
  {
    (uint16_t)500,
    0,
    (int16_t)PHASE5_FINAL_CURRENT,
    (void*)MC_NULL
  },

  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),(uint16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),(uint16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),(uint16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),(uint16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),(uint16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
  },
};

/**
  * @brief  PWM parameters Motor 2 for one ADC.
  */
PWMC_R3_1_Handle_t PWM_Handle_M2 =
{
  {
    .pFctGetPhaseCurrents       = &R3_1_GetPhaseCurrents,
    .pFctSetADCSampPointSectX   = &R3_1_SetADCSampPointSectX,
    .pFctSetOffsetCalib         = &R3_1_SetOffsetCalib,
    .pFctGetOffsetCalib         = &R3_1_GetOffsetCalib,
    .pFctSwitchOffPwm           = &R3_1_SwitchOffPWM,
    .pFctSwitchOnPwm            = &R3_1_SwitchOnPWM,
    .pFctCurrReadingCalib       = &R3_1_CurrentReadingPolarization,
    .pFctTurnOnLowSides         = &R3_1_TurnOnLowSides,
    .pFctOCPSetReferenceVoltage = MC_NULL,
    .pFctRLDetectionModeEnable  = MC_NULL,
    .pFctRLDetectionModeDisable = MC_NULL,
    .pFctRLDetectionModeSetDuty = MC_NULL,
    .pFctRLTurnOnLowSidesAndStart = MC_NULL,
    .hT_Sqrt3                   = (PWM_PERIOD_CYCLES2*SQRT3FACTOR)/16384u,
    .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING2,
    .pwm_en_u_port     = MC_NULL,
    .pwm_en_u_pin      = (uint16_t)0,
    .pwm_en_v_port     = MC_NULL,
    .pwm_en_v_pin      = (uint16_t)0,
    .pwm_en_w_port     = MC_NULL,
    .pwm_en_w_pin      = (uint16_t)0,
    .Sector                     = 0,
    .CntPhA                     = 0,
    .CntPhB                     = 0,
    .CntPhC                     = 0,
    .SWerror                    = 0,
    .TurnOnLowSidesAction       = false,
    .OffCalibrWaitTimeCounter   = 0,
    .Motor                      = M2,
    .RLDetectionMode            = false,
    .Ia                         = 0,
    .Ib                         = 0,
    .Ic                         = 0,
    .LPFIqd_const               = LPF_FILT_CONST,
    .DTCompCnt                  = DTCOMPCNT2,
    .PWMperiod                  = PWM_PERIOD_CYCLES2,
    .Ton                        = TON2,
    .Toff                       = TOFF2,
    .OverCurrentFlag            = false,
    .OverVoltageFlag            = false,
    .BrakeActionLock            = false,
    .driverProtectionFlag       = false,
  },

  .PhaseAOffset                 = 32767,
  .PhaseBOffset                 = 32767,
  .PhaseCOffset                 = 32767,
  .Half_PWMPeriod               = PWM_PERIOD_CYCLES2 / 2u,
  .PolarizationCounter = 0,
  .PolarizationSector = 0,
  .pParams_str                  = &R3_1_ParamsM2
};

PWMC_R3_2_Handle_t PWM_Handle_M1 =
{
  ._Super =
  {
    .pFctGetPhaseCurrents       = &R3_2_GetPhaseCurrents,
    .pFctSetADCSampPointSectX   = &R3_2_SetADCSampPointSectX,
    .pFctSetOffsetCalib         = &R3_2_SetOffsetCalib,
    .pFctGetOffsetCalib         = &R3_2_GetOffsetCalib,
    .pFctSwitchOffPwm           = &R3_2_SwitchOffPWM,
    .pFctSwitchOnPwm            = &R3_2_SwitchOnPWM,
    .pFctCurrReadingCalib       = &R3_2_CurrentReadingPolarization,
    .pFctTurnOnLowSides         = &R3_2_TurnOnLowSides,
    .pFctOCPSetReferenceVoltage = MC_NULL,
    .pFctRLDetectionModeEnable  = &R3_2_RLDetectionModeEnable,
    .pFctRLDetectionModeDisable = &R3_2_RLDetectionModeDisable,
    .pFctRLDetectionModeSetDuty = &R3_2_RLDetectionModeSetDuty,
    .pFctRLTurnOnLowSidesAndStart = &R3_2_RLTurnOnLowSidesAndStart,
    .hT_Sqrt3                   = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
    .LowSideOutputs    = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
    .pwm_en_u_port     = MC_NULL,
    .pwm_en_u_pin      = (uint16_t)0,
    .pwm_en_v_port     = MC_NULL,
    .pwm_en_v_pin      = (uint16_t)0,
    .pwm_en_w_port     = MC_NULL,
    .pwm_en_w_pin      = (uint16_t)0,
    .Sector                     = 0,
    .lowDuty                    = (uint16_t)0,
    .midDuty                    = (uint16_t)0,
    .highDuty                   = (uint16_t)0,
    .CntPhA                     = 0,
    .CntPhB                     = 0,
    .CntPhC                     = 0,
    .SWerror                    = 0,
    .TurnOnLowSidesAction       = false,
    .OffCalibrWaitTimeCounter   = 0,
    .Motor                      = M1,
    .RLDetectionMode            = false,
    .SingleShuntTopology        = false,
    .Ia                         = 0,
    .Ib                         = 0,
    .Ic                         = 0,
    .LPFIqd_const               = LPF_FILT_CONST,
    .DTCompCnt                  = DTCOMPCNT,
    .PWMperiod                  = PWM_PERIOD_CYCLES,
    .Ton                        = TON,
    .Toff                       = TOFF,
    .OverCurrentFlag            = false,
    .OverVoltageFlag            = false,
    .BrakeActionLock            = false,
    .driverProtectionFlag       = false,
  },

  .Half_PWMPeriod               = PWM_PERIOD_CYCLES/2u,
  .PhaseAOffset                 = 32767,
  .PhaseBOffset                 = 32767,
  .PhaseCOffset                 = 32767,
  .PolarizationCounter          = (uint8_t)0,
  .PolarizationSector           = (uint8_t)0,
  .pParams_str                  = &R3_2_ParamsM1
};

/**
  * @brief  PI / PID Speed loop parameters Motor 2.
  */
PID_Handle_t PIDSpeedHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)(IQMAX2 * SP_KIDIV2),
  .wLowerIntegralLimit = -(int32_t)(IQMAX2 * SP_KIDIV2),
  .hUpperOutputLimit   = (int16_t)IQMAX2,
  .hLowerOutputLimit   = -(int16_t)IQMAX2,
  .hKpDivisor          = (uint16_t)SP_KPDIV2,
  .hKiDivisor          = (uint16_t)SP_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG2,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 2.
  */
PID_Handle_t PIDIqHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)(INT16_MAX * TF_KIDIV2),
  .wLowerIntegralLimit = (int32_t)(-INT16_MAX * TF_KIDIV2),
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 2.
  */
PID_Handle_t PIDIdHandle_M2 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
  .hUpperOutputLimit   = INT16_MAX,
  .hLowerOutputLimit   = -INT16_MAX,
  .hKpDivisor          = (uint16_t)TF_KPDIV2,
  .hKiDivisor          = (uint16_t)TF_KIDIV2,
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2.
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE2,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT2),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT2),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT2),
  .MaxPositiveTorque          = (int16_t)NOMINAL_CURRENT2,
  .MinNegativeTorque          = -(int16_t)NOMINAL_CURRENT2,
  .ModeDefault                = DEFAULT_CONTROL_MODE2,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT2),
  .TorqueRefDefault           = (int16_t)DEFAULT_TORQUE_COMPONENT2,
  .IdrefDefault               = (int16_t)DEFAULT_FLUX_COMPONENT2,
};

RevUpCtrl_Handle_t RevUpControlM2 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE2,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG2)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE2-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT2,
  .hMinStartUpFlySpeed     = (int16_t)((OBS_MINIMUM_SPEED_UNIT2)/2),
  .OTFStartupEnabled       = false,

  .OTFPhaseParams =
  {
    (uint16_t)500,
    0,
    (int16_t)PHASE5_FINAL_CURRENT2,
    (void*)MC_NULL
  },

  .ParamsData =
  {
    {(uint16_t)PHASE1_DURATION2,(int16_t)(PHASE1_FINAL_SPEED_UNIT2),(uint16_t)PHASE1_FINAL_CURRENT2,&RevUpControlM2.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION2,(int16_t)(PHASE2_FINAL_SPEED_UNIT2),(uint16_t)PHASE2_FINAL_CURRENT2,&RevUpControlM2.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION2,(int16_t)(PHASE3_FINAL_SPEED_UNIT2),(uint16_t)PHASE3_FINAL_CURRENT2,&RevUpControlM2.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION2,(int16_t)(PHASE4_FINAL_SPEED_UNIT2),(uint16_t)PHASE4_FINAL_CURRENT2,&RevUpControlM2.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION2,(int16_t)(PHASE5_FINAL_SPEED_UNIT2),(uint16_t)PHASE5_FINAL_CURRENT2,(void*)MC_NULL},
  },
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - State Observer + CORDIC.
  */
STO_CR_Handle_t STO_CR_M2 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM2,
    .SpeedUnit                 = SPEED_UNIT,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT2),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .bMaximumSpeedErrorsNumber = M2_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED2,
    .DPPConvFactor             = DPP_CONV_FACTOR2,
  },

  .hC1                         = CORD_C12,
  .hC2                         = CORD_C22,
  .hC3                         = CORD_C32,
  .hC4                         = CORD_C42,
  .hC5                         = CORD_C52,
  .hF1                         = CORD_F12,
  .hF2                         = CORD_F22,
  .SpeedBufferSizeUnit         = CORD_FIFO_DEPTH_UNIT2,
  .SpeedBufferSizedpp          = CORD_FIFO_DEPTH_DPP2,
  .VariancePercentage          = CORD_PERCENTAGE_FACTOR2,
  .SpeedValidationBand_H       = SPEED_BAND_UPPER_LIMIT2,
  .SpeedValidationBand_L       = SPEED_BAND_LOWER_LIMIT2,
  .MinStartUpValidSpeed        = OBS_MINIMUM_SPEED_UNIT2,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS2,
  .MaxInstantElAcceleration    = CORD_MAX_ACCEL_DPPP2,
  .BemfConsistencyCheck        = M2_CORD_BEMF_CONSISTENCY_TOL,
  .BemfConsistencyGain         = M2_CORD_BEMF_CONSISTENCY_GAIN,
  .MaxAppPositiveMecSpeedUnit  = (uint16_t)(MAX_APPLICATION_SPEED_UNIT2 * 1.15),
  .F1LOG                       = CORD_F1_LOG2 ,
  .F2LOG                       = CORD_F2_LOG2 ,
  .SpeedBufferSizedppLOG       = CORD_FIFO_DEPTH_DPP_LOG2
};

STO_Handle_t STO_M2 =
{
  ._Super                        = (SpeednPosFdbk_Handle_t *)&STO_CR_M2, //cstat !MISRAC2012-Rule-11.3
  .pFctForceConvergency1         = &STO_CR_ForceConvergency1,
  .pFctForceConvergency2         = &STO_CR_ForceConvergency2,
  .pFctStoOtfResetPLL            = MC_NULL,
  .pFctSTO_SpeedReliabilityCheck = &STO_CR_IsSpeedReliable
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + CORDIC.
  */
STO_CR_Handle_t STO_CR_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .SpeedUnit                 = SPEED_UNIT,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15 * MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .hC1                         = CORD_C1,
  .hC2                         = CORD_C2,
  .hC3                         = CORD_C3,
  .hC4                         = CORD_C4,
  .hC5                         = CORD_C5,
  .hF1                         = CORD_F1,
  .hF2                         = CORD_F2,
  .SpeedBufferSizeUnit         = CORD_FIFO_DEPTH_UNIT,
  .SpeedBufferSizedpp          = CORD_FIFO_DEPTH_DPP,
  .VariancePercentage          = CORD_PERCENTAGE_FACTOR,
  .SpeedValidationBand_H       = SPEED_BAND_UPPER_LIMIT,
  .SpeedValidationBand_L       = SPEED_BAND_LOWER_LIMIT,
  .MinStartUpValidSpeed        = OBS_MINIMUM_SPEED_UNIT,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS,
  .MaxInstantElAcceleration    = CORD_MAX_ACCEL_DPPP,
  .BemfConsistencyCheck        = M1_CORD_BEMF_CONSISTENCY_TOL,
  .BemfConsistencyGain         = M1_CORD_BEMF_CONSISTENCY_GAIN,
  .MaxAppPositiveMecSpeedUnit  = (uint16_t)(MAX_APPLICATION_SPEED_UNIT * 1.15),
  .F1LOG                       = CORD_F1_LOG,
  .F2LOG                       = CORD_F2_LOG,
  .SpeedBufferSizedppLOG       = CORD_FIFO_DEPTH_DPP_LOG
};

STO_Handle_t STO_M1 =
{
  ._Super                        = (SpeednPosFdbk_Handle_t *)&STO_CR_M1, //cstat !MISRAC2012-Rule-11.3
  .pFctForceConvergency1         = &STO_CR_ForceConvergency1,
  .pFctForceConvergency2         = &STO_CR_ForceConvergency2,
  .pFctStoOtfResetPLL            = MC_NULL,
  .pFctSTO_SpeedReliabilityCheck = &STO_CR_IsSpeedReliable
};

/** RAMP for Motor1
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component.
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule = MAX_MODULE,
  .MaxVd     = (uint16_t)((MAX_MODULE * 950) / 1000),
};

/** RAMP for Motor2
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM2 =
{
  .FrequencyHz = TF_REGULATION_RATE2
};

/**
  * @brief  CircleLimitation Component parameters Motor 2 - Base Component.
  */
CircleLimitation_Handle_t CircleLimitationM2 =
{
  .MaxModule = MAX_MODULE2,
  .MaxVd     = (uint16_t)((MAX_MODULE2 * 950) / 1000),
};

FOCVars_t FOCVars[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1 ,&SpeednTorqCtrlM2};
PID_Handle_t *pPIDIq[NBR_OF_MOTORS]             = {&PIDIqHandle_M1 ,&PIDIqHandle_M2};
PID_Handle_t *pPIDId[NBR_OF_MOTORS]             = {&PIDIdHandle_M1 ,&PIDIdHandle_M2};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1 ,&TempSensor_M2};
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS]  = {&PQD_MotorPowMeasM1,&PQD_MotorPowMeasM2};

MCI_Handle_t Mci[NBR_OF_MOTORS] =
{
  {
    .pSTC = &SpeednTorqCtrlM1,
    .pFOCVars = &FOCVars[0],
    .pPWM = &PWM_Handle_M1._Super,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M1,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },

  {
    .pSTC = &SpeednTorqCtrlM2,
    .pFOCVars = &FOCVars[1],
    .pPWM = &PWM_Handle_M2._Super,
    .lastCommand = MCI_NOCOMMANDSYET,
    .hFinalSpeed = 0,
    .hFinalTorque = 0,
    .pScale = &scaleParams_M2,
    .hDurationms = 0,
    .DirectCommand = MCI_NO_COMMAND,
    .State = IDLE,
    .CurrentFaults = MC_NO_FAULTS,
    .PastFaults = MC_NO_FAULTS,
    .CommandState = MCI_BUFFER_EMPTY,
  },
};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/

