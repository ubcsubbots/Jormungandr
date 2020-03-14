//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: thrust_allocation.h
//
// Code generated for Simulink model 'thrust_allocation'.
//
// Model version                  : 1.76
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Mar 13 18:47:00 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_thrust_allocation_h_
#define RTW_HEADER_thrust_allocation_h_
#include "rtwtypes.h"
#include <cmath>
#include <math.h>
#include <cstring>
#include <stddef.h>
#ifndef thrust_allocation_COMMON_INCLUDES_
# define thrust_allocation_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 // thrust_allocation_COMMON_INCLUDES_

#include <stddef.h>
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Forward declaration for rtModel
typedef struct tag_RTM_thrust_allocation_T RT_MODEL_thrust_allocation_T;

#ifndef DEFINED_TYPEDEF_FOR_struct_7nVZUHcjFLyofg69uDOw4_
#define DEFINED_TYPEDEF_FOR_struct_7nVZUHcjFLyofg69uDOw4_

typedef struct {
  real_T m;
  real_T V;
  real_T W;
  real_T B;
  real_T xg;
  real_T yg;
  real_T zg;
  real_T xb;
  real_T yb;
  real_T zb;
  real_T Ixx;
  real_T Iyy;
  real_T Izz;
  real_T Xu;
  real_T Yv;
  real_T Zw;
  real_T Kp;
  real_T Kq;
  real_T Nr;
  real_T Xuu;
  real_T Yvv;
  real_T Zww;
  real_T Kpp;
  real_T Kqq;
  real_T Nrr;
  real_T alpha;
  real_T L1x;
  real_T L1y;
  real_T L2x;
  real_T L2y;
  real_T L3x;
  real_T L3y;
  real_T L4x;
  real_T L4y;
  real_T L5x;
  real_T L6x;
} struct_7nVZUHcjFLyofg69uDOw4;

#endif

// Real-time Model Data Structure
struct tag_RTM_thrust_allocation_T {
  const char_T **errorStatus;
};

// Class declaration for model thrust_allocation
class thrust_allocationModelClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(const real_T arg_tau_[6], real_T arg_tau_actual_[6], real_T
            arg_signals_[6], real_T arg_thrust[6], real_T *arg_I_sum_);

  // Constructor
  thrust_allocationModelClass();

  // Destructor
  ~thrust_allocationModelClass();

  // Real-Time Model get method
  RT_MODEL_thrust_allocation_T * getRTM();

  //member function to setup error status pointer
  void setErrorStatusPointer(const char_T **rt_errorStatus);

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL_thrust_allocation_T thrust_allocation_M;
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
//  '<Root>' : 'thrust_allocation'
//  '<S1>'   : 'thrust_allocation/thrust_allocator'

#endif                                 // RTW_HEADER_thrust_allocation_h_

//
// File trailer for generated code.
//
// [EOF]
//
