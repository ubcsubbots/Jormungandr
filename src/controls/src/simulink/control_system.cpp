//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: control_system.cpp
//
// Code generated for Simulink model 'control_system'.
//
// Model version                  : 1.14
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Mar 13 18:50:23 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "control_system.h"

static void rate_scheduler(RT_MODEL_control_system_T *const control_system_M);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_control_system_T *const control_system_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (control_system_M->Timing.TaskCounters.TID[1])++;
  if ((control_system_M->Timing.TaskCounters.TID[1]) > 4) {// Sample time: [1.0s, 0.0s] 
    control_system_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function
void control_system2ModelClass::step()
{
  // local block i/o variables
  real_T rtb_Model_o1[6];
  real_T rtb_Model_o2[6];
  real_T rtb_Model_o3[6];
  real_T rtb_Ma_bCnunuDnunugeta[6];
  real_T rtb_tau_actual[6];
  real_T rtb_thruster_signals[6];
  real_T rtb_thrust[6];
  real_T rtb_I_sum;
  int32_T i;

  // ModelReference: '<Root>/Model' incorporates:
  //   Inport: '<Root>/endpoint '
  //   Inport: '<Root>/time '

  ModelMDLOBJ1.step(control_system_U.time, &control_system_U.endpoint[0],
                    &rtb_Model_o1[0], &rtb_Model_o2[0], &rtb_Model_o3[0]);

  // ModelReference: '<Root>/Model1' incorporates:
  //   Inport: '<Root>/x'

  Model1MDLOBJ2.step(&rtb_Model_o1[0], &rtb_Model_o3[0], &control_system_U.x[0],
                     &rtb_Ma_bCnunuDnunugeta[0]);

  // Saturate: '<Root>/Thruster Limit'
  for (i = 0; i < 6; i++) {
    if (rtb_Ma_bCnunuDnunugeta[i] > 10.0) {
      control_system_DW.ThrusterLimit[i] = 10.0;
    } else if (rtb_Ma_bCnunuDnunugeta[i] < -10.0) {
      control_system_DW.ThrusterLimit[i] = -10.0;
    } else {
      control_system_DW.ThrusterLimit[i] = rtb_Ma_bCnunuDnunugeta[i];
    }
  }

  // End of Saturate: '<Root>/Thruster Limit'
  if ((&control_system_M)->Timing.TaskCounters.TID[1] == 0) {
    // ModelReference: '<Root>/Model2'
    Model2MDLOBJ3.step(&control_system_DW.ThrusterLimit[0], &rtb_tau_actual[0],
                       &rtb_thruster_signals[0], &rtb_thrust[0], &rtb_I_sum);
    for (i = 0; i < 6; i++) {
      // Outport: '<Root>/signals'
      control_system_Y.signals[i] = rtb_thruster_signals[i];

      // Outport: '<Root>/tau_actual'
      control_system_Y.tau_actual[i] = rtb_tau_actual[i];
    }
  }

  rate_scheduler((&control_system_M));
}

// Model initialize function
void control_system2ModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // Model Initialize function for ModelReference Block: '<Root>/Model'

  // Set error status pointer for ModelReference Block: '<Root>/Model'
  ModelMDLOBJ1.setErrorStatusPointer(rtmGetErrorStatusPointer((&control_system_M)));
  ModelMDLOBJ1.initialize();

  // Model Initialize function for ModelReference Block: '<Root>/Model1'

  // Set error status pointer for ModelReference Block: '<Root>/Model1'
  Model1MDLOBJ2.setErrorStatusPointer(rtmGetErrorStatusPointer
    ((&control_system_M)));
  Model1MDLOBJ2.initialize();

  // Model Initialize function for ModelReference Block: '<Root>/Model2'

  // Set error status pointer for ModelReference Block: '<Root>/Model2'
  Model2MDLOBJ3.setErrorStatusPointer(rtmGetErrorStatusPointer
    ((&control_system_M)));
  Model2MDLOBJ3.initialize();

  // Start for ModelReference: '<Root>/Model' incorporates:
  //   Inport: '<Root>/endpoint '
  //   Inport: '<Root>/time '

  ModelMDLOBJ1.start();
}

// Constructor
control_system2ModelClass::control_system2ModelClass() : control_system_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
control_system2ModelClass::~control_system2ModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_control_system_T * control_system2ModelClass::getRTM()
{
  return (&control_system_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
