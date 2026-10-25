
/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
  * @ingroup MCTasksFOC
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

 /** @defgroup ConvFOC Conventional FOC
  *
  * @brief
  *
  * @{
  */

  /** @defgroup MCCockpit MC Cockpit
  *
  * @brief
  *
  * @{
  */

/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup	MCCockpit
  * @{
  */

/** @addtogroup	MCTasksFOC
  * @{
  */

/** @defgroup MCTasksFOC Motor Control Tasks for FOC
  *
  * @brief FOC legacy Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* Angle and Speed estimation for STOPLL and STOCORDIC deactivation for IDLE=0, FAULT_NOW=10, FAULT_OVER=11
and CHARGE_BOOT_CAP=16 */
bool AngleSpeedEstimation[STATE_ENUM_COUNT]={false, true, true, true, true, true, true, true, true, true, false, \
                              false, true, true, true, true, false, true, true, true, true, true, true};

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);
static volatile uint16_t hMFTaskCounterM2 = ((uint16_t)0);
static volatile uint16_t hBootCapDelayCounterM2 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM2 = ((uint16_t)0);

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MediumFrequencyTaskM2(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
static uint16_t FOC_CurrControllerM2(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);
    pwmcHandle[M2] = &PWM_Handle_M2._Super;
    R3_1_Init(&PWM_Handle_M2);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    STO_CR_Init (&STO_CR_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_CR_M1._Super);

    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    STC_Clear(pSTC[M1]);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /******************************************************/
    /*   Motor 2 features initialization                  */
    /******************************************************/

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M2);

    /***********************************************************/
    /*   Main speed  sensor initialization: speed regulation   */
    /***********************************************************/
    STO_CR_Init (&STO_CR_M2);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M2], &PIDSpeedHandle_M2, &STO_CR_M2._Super);

    /****************************************************/
    /*   Rev-up component initialization                */
    /****************************************************/
    RUC_Init(&RevUpControlM2, pSTC[M2], &VirtualSpeedSensorM2, &STO_M2, pwmcHandle[M2]); /* Only if sensorless */

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M2);
    PID_HandleInit(&PIDIdHandle_M2);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M2]->pVBS = &(BusVoltageSensor_M2._Super);
    pMPM[M2]->pFOCVars = &FOCVars[M2];

    pREMNG[M2] = &RampExtMngrHFParamsM2;
    REMNG_Init(pREMNG[M2]);
    FOC_Clear(M2);
    STC_Clear(pSTC[M2]);
    FOCVars[M2].bDriveInput = EXTERNAL;
    FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);
    FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).d;
    MCI_ExecSpeedRamp(&Mci[M2],
    STC_GetMecSpeedRefUnitDefault(pSTC[M2]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  if (M1 == motor)
  {
    R3_2_SwitchOffPWM(pwmcHandle[motor]);
  }
  else
  {
    R3_1_SwitchOffPWM(pwmcHandle[motor]);
  }

  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);

 if (M1 == motor)
  {
    TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  }
  else
  {
    TSK_SetStopPermanencyTimeM2(STOPPERMANENCY_TICKS);
  }
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)STO_CR_CalcAvrgMecSpeedUnit(&STO_CR_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);

              R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              STO_CR_Clear(&STO_CR_M1);

              FOC_Clear( M1 );

                Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
            if(! RUC_Exec(&RevUpControlM1))
            {
            /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
              IqdRef.d = FOCVars[M1].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M1].Iqdref = IqdRef;
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
              ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M1, hForcedMecSpeedUnit);

              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
            }
            else
            {
              ObserverConverged = false;
            }
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(&STO_CR_M1._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M1]);
              (void)REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);

              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            int16_t hForcedMecSpeedUnit;

            /* Compute the virtual speed and positions of the rotor.
               The function returns true if the virtual speed is in the reliability range */
            bool FlagEnableClosedLoop = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
            /* Check if the transition ramp has completed. */
            bool FlagTransitionPhaseCompleted = VSS_TransitionEnded(&VirtualSpeedSensorM1);
            FlagEnableClosedLoop = FlagEnableClosedLoop || FlagTransitionPhaseCompleted;

            /* If any of the above conditions is true, the loop is considered closed.
               The state machine transitions to the RUN state */
            if (true == FlagEnableClosedLoop)
            {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
              PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
              PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                  (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                  / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
              /* USER CODE BEGIN MediumFrequencyTask M1 1 */

              /* USER CODE END MediumFrequencyTask M1 1 */
              STC_SetSpeedSensor(pSTC[M1], &STO_CR_M1._Super); /* Observer has converged */
              FOC_InitAdditionalMethods(M1);
              FOC_CalcCurrRef(M1);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M1].State = RUN;
            }
            else if ((FlagTransitionPhaseCompleted == true) && (FlagEnableClosedLoop == false))
            {
              /* The transition time from Open-Loop to Close-Loop allowed has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);
              if(!SPD_Check((SpeednPosFdbk_Handle_t *)&STO_CR_M1))
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;

  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 2 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM2(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M2 0 */

  /* USER CODE END MediumFrequencyTask M2 0 */

  int16_t wAux = 0;

  (void)STO_CR_CalcAvrgMecSpeedUnit(&STO_CR_M2, &wAux);
  PQD_CalcElMotorPower(pMPM[M2]);

  if (MCI_GetCurrentFaults(&Mci[M2]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M2]) == MC_NO_FAULTS)
    {
      switch (Mci[M2].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M2].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand))
          {
              RUC_Clear(&RevUpControlM2, MCI_GetImposedMotorDirection(&Mci[M2]));

            if (pwmcHandle[M2]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_START);
              Mci[M2].State = OFFSET_CALIB;
            }
            else
            {
             /* Calibration already done. Enables only TIM channels */
             pwmcHandle[M2]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC);

              R3_1_TurnOnLowSides(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM2(M2_CHARGE_BOOT_CAP_TICKS);
              Mci[M2].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M2], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M2].DirectCommand)
              {
                FOC_Clear(M2);
		STC_Clear(pSTC[M2]);
                Mci[M2].DirectCommand = MCI_NO_COMMAND;
                Mci[M2].State = IDLE;
              }
              else
                {

                R3_1_TurnOnLowSides(pwmcHandle[M2],M2_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM2(M2_CHARGE_BOOT_CAP_TICKS);
                Mci[M2].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM2())
            {
              R3_1_SwitchOffPWM(pwmcHandle[M2]);
              FOCVars[M2].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
              STO_CR_Clear(&STO_CR_M2);
		FOC_Clear( M2 );

                Mci[M2].State = START;

              PWMC_SwitchOnPWM(pwmcHandle[M2]);

            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
            if(! RUC_Exec(&RevUpControlM2))
            {
            /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M2], MC_START_UP, 0);
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M2]);
              IqdRef.d = FOCVars[M2].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M2].Iqdref = IqdRef;
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);

            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM2))
            {
              ObserverConverged = STO_CR_IsObserverConverged(&STO_CR_M2, hForcedMecSpeedUnit);

              (void)VSS_SetStartTransition(&VirtualSpeedSensorM2, ObserverConverged);
            }
            else
            {
              ObserverConverged = false;
            }
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M2].Ialphabeta, SPD_GetElAngle(&STO_CR_M2._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M2]);
              (void)REMNG_ExecRamp(pREMNG[M2], FOCVars[M2].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M2], StatorCurrent.q, TRANSITION_DURATION);

              Mci[M2].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            int16_t hForcedMecSpeedUnit;

            /* Compute the virtual speed and positions of the rotor.
               The function returns true if the virtual speed is in the reliability range */
            bool FlagEnableClosedLoop = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM2, &hForcedMecSpeedUnit);
            /* Check if the transition ramp has completed. */
            bool FlagTransitionPhaseCompleted = VSS_TransitionEnded(&VirtualSpeedSensorM2);
            FlagEnableClosedLoop = FlagEnableClosedLoop || FlagTransitionPhaseCompleted;

            /* If any of the above conditions is true, the loop is considered closed.
               The state machine transitions to the RUN state */
            if (true ==  FlagEnableClosedLoop)
            {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
              PID_SetIntegralTerm(&PIDSpeedHandle_M2, 0);
#else
              PID_SetIntegralTerm(&PIDSpeedHandle_M2,
                                  (((int32_t)FOCVars[M2].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M2))
                                  / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
              /* USER CODE BEGIN MediumFrequencyTask M2 1 */

              /* USER CODE END MediumFrequencyTask M2 1 */
              STC_SetSpeedSensor(pSTC[M2], &STO_CR_M2._Super); /* Observer has converged */
              FOC_InitAdditionalMethods(M2);
              FOC_CalcCurrRef(M2);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M2]); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M2].State = RUN;
            }
            else if ((FlagTransitionPhaseCompleted == true) && (FlagEnableClosedLoop == false))
            {
              /* The transition time from Open-Loop to Close-Loop allowed has expired */
              MCI_FaultProcessing(&Mci[M2], MC_START_UP, 0);
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M2].DirectCommand)
          {
            TSK_MF_StopProcessing(M2);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M2 2 */

            /* USER CODE END MediumFrequencyTask M2 2 */

            MCI_ExecBufferedCommands(&Mci[M2]);

              FOC_CalcCurrRef(M2);

              if(!SPD_Check((SpeednPosFdbk_Handle_t *) &STO_CR_M2))
              {
                MCI_FaultProcessing(&Mci[M2], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM2())
          {

            STC_SetSpeedSensor(pSTC[M2], &VirtualSpeedSensorM2._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM2); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M2 5 */

            /* USER CODE END MediumFrequencyTask M2 5 */
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M2].DirectCommand)
          {
            Mci[M2].DirectCommand = MCI_NO_COMMAND;
            Mci[M2].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M2].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M2].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M2].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M2 6 */

  /* USER CODE END MediumFrequencyTask M2 6 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  * @param bMotorNbr Motor reference number defined
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */

  if (M1 == bMotorNbr)
  {
    RCM_ReadOngoingConv();
    RCM_ExecNextConv();

    STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;        /* Only if motor0 is sensorless */
    if (SWITCH_OVER == Mci[M1].State)
    {
      if (!REMNG_RampCompleted(pREMNG[M1]))
      {
        FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_1 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_1 */
    hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_2 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_2 */
  }
  else /* bMotorNbr != M1 */
  {
    STO_Inputs.Valfa_beta = FOCVars[M2].Valphabeta;      /* Only if motor2 is sensorless */
    if (SWITCH_OVER == Mci[M2].State)
    {
      if (!REMNG_RampCompleted(pREMNG[M2]))
      {
        FOCVars[M2].Iqdref.q = ( int16_t )REMNG_Calc(pREMNG[M2]);
      }
    }
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_3 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_3 */
    hFOCreturn = FOC_CurrControllerM2();
  /* USER CODE BEGIN HighFrequencyTask DUALDRIVE_4 */

  /* USER CODE END HighFrequencyTask DUALDRIVE_4 */
  }
  if(MC_DURATION == hFOCreturn)
  {
    MCI_FaultProcessing(&Mci[bMotorNbr], MC_DURATION, 0);
  }
  else
  {
    if (M1 == bMotorNbr)
    {
      if (true == AngleSpeedEstimation[Mci[M1].State])
      {
        STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
        STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
        (void)STO_CR_CalcElAngle(&STO_CR_M1, &STO_Inputs);
        STO_CR_CalcAvrgElSpeedDpp(&STO_CR_M1); /* Only in case of Sensor-less */
      }
      else
      {
        /* Nothing to do */
      }

      /* Only for sensor-less */
      if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State) || (RUN == Mci[M1].State))
      {
        int16_t hObsAngle = SPD_GetElAngle(&STO_CR_M1._Super);
        (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else /* bMotorNbr != M1 */
    {
      if (true == AngleSpeedEstimation[Mci[M2].State])
      {
        STO_Inputs.Ialfa_beta = FOCVars[M2].Ialphabeta; /* Only if sensorless */
        STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M2._Super)); /* Only for sensorless */
        (void)STO_CR_CalcElAngle(&STO_CR_M2, &STO_Inputs);
        STO_CR_CalcAvrgElSpeedDpp(&STO_CR_M2); /* Only in case of Sensor-less */

      }
      else
      {
        /* Nothing to do */
      }

      /* Only for sensor-less */
      if((START == Mci[M2].State) || (SWITCH_OVER == Mci[M2].State) || (RUN == Mci[M2].State))
      {
        int16_t hObsAngle = SPD_GetElAngle(&STO_CR_M2._Super);
        (void)VSS_CalcElAngle(&VirtualSpeedSensorM2, &hObsAngle);
      }
      else
      {
        /* Nothing to do */
      }

    }
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

  }

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  return (hCodeError);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation of motor 2. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted.
  * @param This related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise.
  */
inline uint16_t FOC_CurrControllerM2(void)
{
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  qd_t Iqd, Vqd;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M2]);
  hElAngle = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M2], &Iab);
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  if (PWMC_GetPWMState(pwmcHandle[M2]) == true)
  {
    Vqd.q = PI_Controller(pPIDIq[M2], (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M2], (int32_t)(FOCVars[M2].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }

  Vqd = Circle_Limitation(&CircleLimitationM2, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M2]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M2], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */
  }

  FOCVars[M2].Vqd = Vqd;
  FOCVars[M2].Iab = Iab;
  FOCVars[M2].Ialphabeta = Ialphabeta;
  FOCVars[M2].Iqd = Iqd;
  FOCVars[M2].Valphabeta = Valphabeta;
  FOCVars[M2].hElAngle = hElAngle;
  return(hCodeError);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
