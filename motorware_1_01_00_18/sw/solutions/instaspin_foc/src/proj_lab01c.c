/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab01c.c
//! \brief  current close loop control to check hardware
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB01c PROJ_LAB01c
//@{

//! \defgroup PROJ_LAB01c_OVERVIEW Project Overview
//!
//! Closed current loop, open speed loop testing for signal chain integrity
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5


// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif
HAL_Handle halHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif
USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
#ifdef CSM_ENABLE
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif
CTRL_Obj *controller_obj;		//
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

// define CPU time
CPU_TIME_Handle  cpu_timeHandle;
CPU_TIME_Obj     cpu_time;

#ifndef F2802xF
HAL_DacData_t gDacData;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

// define Angle Generate
ANGLE_GEN_Handle angle_genHandle;
ANGLE_GEN_Obj    angle_gen;

// **************************************************************************
// the functions

void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 1;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);

  #ifdef CSM_ENABLE
    //copy .econst to unsecure RAM
    if(*econst_end - *econst_start)
      {
        memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
      }

    //copy .switch ot unsecure RAM
    if(*switch_end - *switch_start)
      {
        memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
      }
  #endif
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif

  controller_obj = (CTRL_Obj *)ctrlHandle;

  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);

  // initialize the angle generate module
  angle_genHandle = ANGLE_GEN_init(&angle_gen,sizeof(angle_gen));
  ANGLE_GEN_setParams(angle_genHandle, gUserParams.iqFullScaleFreq_Hz, gUserParams.ctrlPeriod_sec);

  // initialize the CPU usage module
  cpu_timeHandle = CPU_TIME_init(&cpu_time,sizeof(cpu_time));
  CPU_TIME_setParams(cpu_timeHandle, PWM_getPeriod(halHandle->pwmHandle[0]));

#ifndef F2802xF
  // set DAC parameters
//  gDacData.ptrData[0] = &angle_gen.Angle_pu;
//  gDacData.ptrData[1] = &gAdcData.I.value[0];
//  gDacData.ptrData[2] = &gAdcData.V.value[0];
//  gDacData.ptrData[3] = &gPwmData.Tabc.value[0];

  gDacData.ptrData[0] = &angle_gen.Angle_pu;
  gDacData.ptrData[1] = &controller_obj->pid_Iq.refValue;
  gDacData.ptrData[2] = &controller_obj->pid_Iq.fbackValue;
  gDacData.ptrData[3] = &gPwmData.Tabc.value[0];

  HAL_setDacParameters(halHandle, &gDacData);
#endif

#ifndef F2802xF
  // Initialize Datalog
  datalogHandle = DATALOG_init(&datalog,sizeof(datalog));
  DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

  // Connect inputs of the datalog module
//  datalog.iptr[0] = &angle_gen.Angle_pu;		// datalogBuff[0]
//  datalog.iptr[1] = &gAdcData.I.value[0];		// datalogBuff[1]
//  datalog.iptr[2] = &gAdcData.V.value[0];		// datalogBuff[2]

  datalog.iptr[0] = &angle_gen.Angle_pu;					// datalogBuff[0]
  datalog.iptr[1] = &controller_obj->pid_Iq.refValue;		// datalogBuff[1]
  datalog.iptr[2] = &controller_obj->pid_Iq.fbackValue;		// datalogBuff[2]

  datalogObj->Flag_EnableLogData = true;
  datalogObj->Flag_EnableLogOneShot = false;
#endif

  // setup faults
  HAL_setupFaults(halHandle);

  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
  // turn on the DRV8305 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8305 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

  gMotorVars.IdSet_A = _IQ(0.0);
  gMotorVars.IqSet_A = _IQ(0.1*USER_MOTOR_MAX_CURRENT);

  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys));

    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2);

                    // set flag to enable speed controller
                   CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);

                   // set flag to enable current controller
                   CTRL_setFlag_enableCurrentCtrl(ctrlHandle,true);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));
            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);

              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle,CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle,CTRL_Type_PID_Id);

              // initialize the watch window kp and ki values with pre-calculated values
              gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle,CTRL_Type_PID_spd);
              gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle,CTRL_Type_PID_spd);
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle);
          }


        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
        HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

        HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif
      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{
  // toggle status LED
  if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);

  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);

  // run the controller
  uint_least16_t count_isr = CTRL_getCount_isr(ctrlHandle);
  uint_least16_t numIsrTicksPerCtrlTick = CTRL_getNumIsrTicksPerCtrlTick(ctrlHandle);

  // if needed, run the controller
  if(count_isr >= numIsrTicksPerCtrlTick)
    {
      CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

      // reset the isr count
      CTRL_resetCounter_isr(ctrlHandle);

      // increment the state counter
      CTRL_incrCounter_state(ctrlHandle);

      // increment the trajectory count
      CTRL_incrCounter_traj(ctrlHandle);

      // run the appropriate controller
      if(ctrlState == CTRL_State_OnLine)
        {
          // increment the current count
          CTRL_incrCounter_current(ctrlHandle);

          // increment the speed count
          CTRL_incrCounter_speed(ctrlHandle);

          MATH_vec2 phasor;

         // run Clarke transform on current
         CLARKE_run(controller_obj->clarkeHandle_I,&gAdcData.I,CTRL_getIab_in_addr(ctrlHandle));

         // run Clarke transform on voltage
         CLARKE_run(controller_obj->clarkeHandle_V,&gAdcData.V,CTRL_getVab_in_addr(ctrlHandle));

         controller_obj->speed_ref_pu = TRAJ_getIntValue(controller_obj->trajHandle_spd);

         ANGLE_GEN_run(angle_genHandle, controller_obj->speed_ref_pu);

         // generate the motor electrical angle
       	 controller_obj->angle_pu = ANGLE_GEN_getAngle_pu(angle_genHandle);

         CTRL_setId_ref_pu(ctrlHandle, gMotorVars.IdRef_pu);
         CTRL_setIq_ref_pu(ctrlHandle, gMotorVars.IqRef_pu);

         // compute the sin/cos phasor
         CTRL_computePhasor(controller_obj->angle_pu,&phasor);

         // set the phasor in the Park transform
         PARK_setPhasor(controller_obj->parkHandle,&phasor);

         // run the Park transform
         PARK_run(controller_obj->parkHandle,CTRL_getIab_in_addr(ctrlHandle),CTRL_getIdq_in_addr(ctrlHandle));

         // when appropriate, run the PID Id and Iq controllers
         if(CTRL_doCurrentCtrl(ctrlHandle))
           {
             _iq refValue;
             _iq fbackValue;
             _iq outMin,outMax;

             // read max voltage vector to set proper limits to current controllers
             _iq maxVsMag = CTRL_getMaxVsMag_pu(ctrlHandle);

             // reset the current count
             CTRL_resetCounter_current(ctrlHandle);

             // ***********************************
             // configure and run the Id controller
             // compute the reference value
             refValue = TRAJ_getIntValue(controller_obj->trajHandle_Id) + CTRL_getId_ref_pu(ctrlHandle);

             // update the Id reference value
             EST_updateId_ref_pu(controller_obj->estHandle,&refValue);

             // get the feedback value
             fbackValue = CTRL_getId_in_pu(ctrlHandle);

             // set minimum and maximum for Id controller output
             outMax = maxVsMag;
             outMin = -outMax;

             // set the minimum and maximum values
             PID_setMinMax(controller_obj->pidHandle_Id,outMin,outMax);

             // run the Id PID controller
             PID_run(controller_obj->pidHandle_Id,refValue,fbackValue,CTRL_getVd_out_addr(ctrlHandle));

             // ***********************************
             // configure and run the Iq controller
             // get the reference value
             if(CTRL_getFlag_enableSpeedCtrl(ctrlHandle))
               {
                 refValue = CTRL_getSpd_out_pu(ctrlHandle);
               }
             else
               {
                 // get the Iq reference value
                 refValue = CTRL_getIq_ref_pu(ctrlHandle);
               }

             // get the feedback value
             fbackValue = CTRL_getIq_in_pu(ctrlHandle);

             // set minimum and maximum for Id controller output
             outMax = _IQsqrt(_IQmpy(maxVsMag,maxVsMag) - _IQmpy(CTRL_getVd_out_pu(ctrlHandle),CTRL_getVd_out_pu(ctrlHandle)));
             outMin = -outMax;

             // set the minimum and maximum values
             PID_setMinMax(controller_obj->pidHandle_Iq,outMin,outMax);

             // run the Iq PID controller
             PID_run(controller_obj->pidHandle_Iq,refValue,fbackValue,CTRL_getVq_out_addr(ctrlHandle));
           }

         // set the phasor in the inverse Park transform
         IPARK_setPhasor(controller_obj->iparkHandle,&phasor);

         // run the inverse Park module
         IPARK_run(controller_obj->iparkHandle,CTRL_getVdq_out_addr(ctrlHandle),CTRL_getVab_out_addr(ctrlHandle));

         // run the space Vector Generator (SVGEN) module
         SVGEN_run(controller_obj->svgenHandle,CTRL_getVab_out_addr(ctrlHandle),&(gPwmData.Tabc));
        }
      else if(ctrlState == CTRL_State_OffLine)
        {
          // run the offline controller
          CTRL_runOffLine(ctrlHandle,halHandle,&gAdcData,&gPwmData);
        }
      else if(ctrlState == CTRL_State_Idle)
        {
          // set all pwm outputs to zero
    	  gPwmData.Tabc.value[0] = _IQ(0.0);
    	  gPwmData.Tabc.value[1] = _IQ(0.0);
    	  gPwmData.Tabc.value[2] = _IQ(0.0);
        }
    }
  else
    {
      // increment the isr count
      CTRL_incrCounter_isr(ctrlHandle);
    }


  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  // setup the controller
  CTRL_setup(ctrlHandle);

#ifndef F2802xF
  DATALOG_update(datalogHandle);
#endif

#ifndef F2802xF
  // connect inputs of the PWMDAC module.
  gDacData.value[0] = (*gDacData.ptrData[0]); 	// Motor_AdcTreat.iqSampleIu;
  gDacData.value[1] = (*gDacData.ptrData[1]); 	// controller_obj->angle_esmo_pu;
  gDacData.value[2] = (*gDacData.ptrData[2]); 	// controller_obj->angle_est_pu;
  gDacData.value[3] = (*gDacData.ptrData[3]); 	// controller_obj->angle_ext_pu;

  HAL_writeDacData(halHandle,&gDacData);
#endif

  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  gMotorVars.IdRef_pu = _IQdiv(gMotorVars.IdSet_A, _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
  gMotorVars.IqRef_pu = _IQdiv(gMotorVars.IqSet_A, _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function


//@} //defgroup
// end of file



