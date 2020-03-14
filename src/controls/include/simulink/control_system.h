//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: control_system.h
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
#ifndef RTW_HEADER_control_system_h_
#define RTW_HEADER_control_system_h_
#ifndef control_system_COMMON_INCLUDES_
# define control_system_COMMON_INCLUDES_
#include "rtwtypes.h"
// #include "rtw_continuous.h"
// #include "rtw_solver.h"
#endif                                 // control_system_COMMON_INCLUDES_

// Child system includes
#include "trajectory_planner.h"
#include "controller.h"
#include "thrust_allocation.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
# define rtmGetErrorStatusPointer(rtm) ((const char_T **)(&((rtm)->errorStatus)))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM_control_system_T RT_MODEL_control_system_T;

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real_T ThrusterLimit[6];             // '<Root>/Thruster Limit'
} DW_control_system_T;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T time;                         // '<Root>/time '
  real_T endpoint[6];                  // '<Root>/endpoint '
  real_T x[12];                        // '<Root>/x'
} ExtU_control_system_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T signals[6];                   // '<Root>/signals'
  real_T tau_actual[6];                // '<Root>/tau_actual'
} ExtY_control_system_T;

// Real-time Model Data Structure
struct tag_RTM_control_system_T {
  const char_T * volatile errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint32_T TID[2];
    } TaskCounters;
  } Timing;
};

// Class declaration for model control_system
class control_system2ModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_control_system_T control_system_U;

  // External outputs
  ExtY_control_system_T control_system_Y;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  control_system2ModelClass();

  // Destructor
  ~control_system2ModelClass();

  // Real-Time Model get method
  RT_MODEL_control_system_T * getRTM();

  // private data and function members
 private:
  // Block signals and states
  DW_control_system_T control_system_DW;

  // Real-Time Model
  RT_MODEL_control_system_T control_system_M;

  // model instance variable for '<Root>/Model'
  trajectory_plannerModelClass ModelMDLOBJ1;

  // model instance variable for '<Root>/Model1'
  controllerModelClass Model1MDLOBJ2;

  // model instance variable for '<Root>/Model2'
  thrust_allocationModelClass Model2MDLOBJ3;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'control_system'

#endif                                 // RTW_HEADER_control_system_h_

//
// File trailer for generated code.
//
// [EOF]
//
