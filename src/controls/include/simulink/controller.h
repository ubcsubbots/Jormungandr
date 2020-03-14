//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller.h
//
// Code generated for Simulink model 'controller'.
//
// Model version                  : 1.72
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Mar 13 18:49:31 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_controller_h_
#define RTW_HEADER_controller_h_
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <stddef.h>
#ifndef controller_COMMON_INCLUDES_
# define controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 // controller_COMMON_INCLUDES_

// Child system includes
#include "frame_conversion.h"
#include <stddef.h>

// Forward declaration for rtModel
typedef struct tag_RTM_controller_T RT_MODEL_controller_T;

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

// Block signals and states (default storage) for model 'controller'
typedef struct {
  real_T Filter_DSTATE;                // '<S165>/Filter'
  real_T Filter_DSTATE_k;              // '<S209>/Filter'
  real_T Integrator_DSTATE;            // '<S258>/Integrator'
  real_T Filter_DSTATE_i;              // '<S253>/Filter'
  real_T Filter_DSTATE_d;              // '<S33>/Filter'
  real_T Filter_DSTATE_a;              // '<S121>/Filter'
  real_T Filter_DSTATE_l;              // '<S77>/Filter'
} DW_controller_T;

// Real-time Model Data Structure
struct tag_RTM_controller_T {
  const char_T **errorStatus;
};

// Class declaration for model controller
class controllerModelClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(const real_T arg_eta_d[6], const real_T arg_eta_d_dot_dot[6], const
            real_T arg_x[12], real_T arg_tau_[6]);

  // Constructor
  controllerModelClass();

  // Destructor
  ~controllerModelClass();

  // Real-Time Model get method
  RT_MODEL_controller_T * getRTM();

  //member function to setup error status pointer
  void setErrorStatusPointer(const char_T **rt_errorStatus);

  // private data and function members
 private:
  // Block signals and states
  DW_controller_T controller_DW;

  // Real-Time Model
  RT_MODEL_controller_T controller_M;

  // model instance variable for '<Root>/Model'
  frame_conversionModelClass ModelMDLOBJ1;
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
//  '<Root>' : 'controller'
//  '<S1>'   : 'controller/C(nu)*nu + D(nu)*nu + g(eta)'
//  '<S2>'   : 'controller/M*J^-1(eta)*(a_n - J_dot(eta, eta_dot)*nu)'
//  '<S3>'   : 'controller/phi PID '
//  '<S4>'   : 'controller/psi PID '
//  '<S5>'   : 'controller/theta PID '
//  '<S6>'   : 'controller/x PID'
//  '<S7>'   : 'controller/y PID'
//  '<S8>'   : 'controller/z PID '
//  '<S9>'   : 'controller/phi PID /Anti-windup'
//  '<S10>'  : 'controller/phi PID /D Gain'
//  '<S11>'  : 'controller/phi PID /Filter'
//  '<S12>'  : 'controller/phi PID /Filter ICs'
//  '<S13>'  : 'controller/phi PID /I Gain'
//  '<S14>'  : 'controller/phi PID /Ideal P Gain'
//  '<S15>'  : 'controller/phi PID /Ideal P Gain Fdbk'
//  '<S16>'  : 'controller/phi PID /Integrator'
//  '<S17>'  : 'controller/phi PID /Integrator ICs'
//  '<S18>'  : 'controller/phi PID /N Copy'
//  '<S19>'  : 'controller/phi PID /N Gain'
//  '<S20>'  : 'controller/phi PID /P Copy'
//  '<S21>'  : 'controller/phi PID /Parallel P Gain'
//  '<S22>'  : 'controller/phi PID /Reset Signal'
//  '<S23>'  : 'controller/phi PID /Saturation'
//  '<S24>'  : 'controller/phi PID /Saturation Fdbk'
//  '<S25>'  : 'controller/phi PID /Sum'
//  '<S26>'  : 'controller/phi PID /Sum Fdbk'
//  '<S27>'  : 'controller/phi PID /Tracking Mode'
//  '<S28>'  : 'controller/phi PID /Tracking Mode Sum'
//  '<S29>'  : 'controller/phi PID /postSat Signal'
//  '<S30>'  : 'controller/phi PID /preSat Signal'
//  '<S31>'  : 'controller/phi PID /Anti-windup/Disabled'
//  '<S32>'  : 'controller/phi PID /D Gain/Internal Parameters'
//  '<S33>'  : 'controller/phi PID /Filter/Disc. Forward Euler Filter'
//  '<S34>'  : 'controller/phi PID /Filter ICs/Internal IC - Filter'
//  '<S35>'  : 'controller/phi PID /I Gain/Disabled'
//  '<S36>'  : 'controller/phi PID /Ideal P Gain/Passthrough'
//  '<S37>'  : 'controller/phi PID /Ideal P Gain Fdbk/Disabled'
//  '<S38>'  : 'controller/phi PID /Integrator/Disabled'
//  '<S39>'  : 'controller/phi PID /Integrator ICs/Disabled'
//  '<S40>'  : 'controller/phi PID /N Copy/Disabled'
//  '<S41>'  : 'controller/phi PID /N Gain/Internal Parameters'
//  '<S42>'  : 'controller/phi PID /P Copy/Disabled'
//  '<S43>'  : 'controller/phi PID /Parallel P Gain/Internal Parameters'
//  '<S44>'  : 'controller/phi PID /Reset Signal/Disabled'
//  '<S45>'  : 'controller/phi PID /Saturation/Passthrough'
//  '<S46>'  : 'controller/phi PID /Saturation Fdbk/Disabled'
//  '<S47>'  : 'controller/phi PID /Sum/Sum_PD'
//  '<S48>'  : 'controller/phi PID /Sum Fdbk/Disabled'
//  '<S49>'  : 'controller/phi PID /Tracking Mode/Disabled'
//  '<S50>'  : 'controller/phi PID /Tracking Mode Sum/Passthrough'
//  '<S51>'  : 'controller/phi PID /postSat Signal/Forward_Path'
//  '<S52>'  : 'controller/phi PID /preSat Signal/Forward_Path'
//  '<S53>'  : 'controller/psi PID /Anti-windup'
//  '<S54>'  : 'controller/psi PID /D Gain'
//  '<S55>'  : 'controller/psi PID /Filter'
//  '<S56>'  : 'controller/psi PID /Filter ICs'
//  '<S57>'  : 'controller/psi PID /I Gain'
//  '<S58>'  : 'controller/psi PID /Ideal P Gain'
//  '<S59>'  : 'controller/psi PID /Ideal P Gain Fdbk'
//  '<S60>'  : 'controller/psi PID /Integrator'
//  '<S61>'  : 'controller/psi PID /Integrator ICs'
//  '<S62>'  : 'controller/psi PID /N Copy'
//  '<S63>'  : 'controller/psi PID /N Gain'
//  '<S64>'  : 'controller/psi PID /P Copy'
//  '<S65>'  : 'controller/psi PID /Parallel P Gain'
//  '<S66>'  : 'controller/psi PID /Reset Signal'
//  '<S67>'  : 'controller/psi PID /Saturation'
//  '<S68>'  : 'controller/psi PID /Saturation Fdbk'
//  '<S69>'  : 'controller/psi PID /Sum'
//  '<S70>'  : 'controller/psi PID /Sum Fdbk'
//  '<S71>'  : 'controller/psi PID /Tracking Mode'
//  '<S72>'  : 'controller/psi PID /Tracking Mode Sum'
//  '<S73>'  : 'controller/psi PID /postSat Signal'
//  '<S74>'  : 'controller/psi PID /preSat Signal'
//  '<S75>'  : 'controller/psi PID /Anti-windup/Disabled'
//  '<S76>'  : 'controller/psi PID /D Gain/Internal Parameters'
//  '<S77>'  : 'controller/psi PID /Filter/Disc. Forward Euler Filter'
//  '<S78>'  : 'controller/psi PID /Filter ICs/Internal IC - Filter'
//  '<S79>'  : 'controller/psi PID /I Gain/Disabled'
//  '<S80>'  : 'controller/psi PID /Ideal P Gain/Passthrough'
//  '<S81>'  : 'controller/psi PID /Ideal P Gain Fdbk/Disabled'
//  '<S82>'  : 'controller/psi PID /Integrator/Disabled'
//  '<S83>'  : 'controller/psi PID /Integrator ICs/Disabled'
//  '<S84>'  : 'controller/psi PID /N Copy/Disabled'
//  '<S85>'  : 'controller/psi PID /N Gain/Internal Parameters'
//  '<S86>'  : 'controller/psi PID /P Copy/Disabled'
//  '<S87>'  : 'controller/psi PID /Parallel P Gain/Internal Parameters'
//  '<S88>'  : 'controller/psi PID /Reset Signal/Disabled'
//  '<S89>'  : 'controller/psi PID /Saturation/Passthrough'
//  '<S90>'  : 'controller/psi PID /Saturation Fdbk/Disabled'
//  '<S91>'  : 'controller/psi PID /Sum/Sum_PD'
//  '<S92>'  : 'controller/psi PID /Sum Fdbk/Disabled'
//  '<S93>'  : 'controller/psi PID /Tracking Mode/Disabled'
//  '<S94>'  : 'controller/psi PID /Tracking Mode Sum/Passthrough'
//  '<S95>'  : 'controller/psi PID /postSat Signal/Forward_Path'
//  '<S96>'  : 'controller/psi PID /preSat Signal/Forward_Path'
//  '<S97>'  : 'controller/theta PID /Anti-windup'
//  '<S98>'  : 'controller/theta PID /D Gain'
//  '<S99>'  : 'controller/theta PID /Filter'
//  '<S100>' : 'controller/theta PID /Filter ICs'
//  '<S101>' : 'controller/theta PID /I Gain'
//  '<S102>' : 'controller/theta PID /Ideal P Gain'
//  '<S103>' : 'controller/theta PID /Ideal P Gain Fdbk'
//  '<S104>' : 'controller/theta PID /Integrator'
//  '<S105>' : 'controller/theta PID /Integrator ICs'
//  '<S106>' : 'controller/theta PID /N Copy'
//  '<S107>' : 'controller/theta PID /N Gain'
//  '<S108>' : 'controller/theta PID /P Copy'
//  '<S109>' : 'controller/theta PID /Parallel P Gain'
//  '<S110>' : 'controller/theta PID /Reset Signal'
//  '<S111>' : 'controller/theta PID /Saturation'
//  '<S112>' : 'controller/theta PID /Saturation Fdbk'
//  '<S113>' : 'controller/theta PID /Sum'
//  '<S114>' : 'controller/theta PID /Sum Fdbk'
//  '<S115>' : 'controller/theta PID /Tracking Mode'
//  '<S116>' : 'controller/theta PID /Tracking Mode Sum'
//  '<S117>' : 'controller/theta PID /postSat Signal'
//  '<S118>' : 'controller/theta PID /preSat Signal'
//  '<S119>' : 'controller/theta PID /Anti-windup/Disabled'
//  '<S120>' : 'controller/theta PID /D Gain/Internal Parameters'
//  '<S121>' : 'controller/theta PID /Filter/Disc. Forward Euler Filter'
//  '<S122>' : 'controller/theta PID /Filter ICs/Internal IC - Filter'
//  '<S123>' : 'controller/theta PID /I Gain/Disabled'
//  '<S124>' : 'controller/theta PID /Ideal P Gain/Passthrough'
//  '<S125>' : 'controller/theta PID /Ideal P Gain Fdbk/Disabled'
//  '<S126>' : 'controller/theta PID /Integrator/Disabled'
//  '<S127>' : 'controller/theta PID /Integrator ICs/Disabled'
//  '<S128>' : 'controller/theta PID /N Copy/Disabled'
//  '<S129>' : 'controller/theta PID /N Gain/Internal Parameters'
//  '<S130>' : 'controller/theta PID /P Copy/Disabled'
//  '<S131>' : 'controller/theta PID /Parallel P Gain/Internal Parameters'
//  '<S132>' : 'controller/theta PID /Reset Signal/Disabled'
//  '<S133>' : 'controller/theta PID /Saturation/Passthrough'
//  '<S134>' : 'controller/theta PID /Saturation Fdbk/Disabled'
//  '<S135>' : 'controller/theta PID /Sum/Sum_PD'
//  '<S136>' : 'controller/theta PID /Sum Fdbk/Disabled'
//  '<S137>' : 'controller/theta PID /Tracking Mode/Disabled'
//  '<S138>' : 'controller/theta PID /Tracking Mode Sum/Passthrough'
//  '<S139>' : 'controller/theta PID /postSat Signal/Forward_Path'
//  '<S140>' : 'controller/theta PID /preSat Signal/Forward_Path'
//  '<S141>' : 'controller/x PID/Anti-windup'
//  '<S142>' : 'controller/x PID/D Gain'
//  '<S143>' : 'controller/x PID/Filter'
//  '<S144>' : 'controller/x PID/Filter ICs'
//  '<S145>' : 'controller/x PID/I Gain'
//  '<S146>' : 'controller/x PID/Ideal P Gain'
//  '<S147>' : 'controller/x PID/Ideal P Gain Fdbk'
//  '<S148>' : 'controller/x PID/Integrator'
//  '<S149>' : 'controller/x PID/Integrator ICs'
//  '<S150>' : 'controller/x PID/N Copy'
//  '<S151>' : 'controller/x PID/N Gain'
//  '<S152>' : 'controller/x PID/P Copy'
//  '<S153>' : 'controller/x PID/Parallel P Gain'
//  '<S154>' : 'controller/x PID/Reset Signal'
//  '<S155>' : 'controller/x PID/Saturation'
//  '<S156>' : 'controller/x PID/Saturation Fdbk'
//  '<S157>' : 'controller/x PID/Sum'
//  '<S158>' : 'controller/x PID/Sum Fdbk'
//  '<S159>' : 'controller/x PID/Tracking Mode'
//  '<S160>' : 'controller/x PID/Tracking Mode Sum'
//  '<S161>' : 'controller/x PID/postSat Signal'
//  '<S162>' : 'controller/x PID/preSat Signal'
//  '<S163>' : 'controller/x PID/Anti-windup/Disabled'
//  '<S164>' : 'controller/x PID/D Gain/Internal Parameters'
//  '<S165>' : 'controller/x PID/Filter/Disc. Forward Euler Filter'
//  '<S166>' : 'controller/x PID/Filter ICs/Internal IC - Filter'
//  '<S167>' : 'controller/x PID/I Gain/Disabled'
//  '<S168>' : 'controller/x PID/Ideal P Gain/Passthrough'
//  '<S169>' : 'controller/x PID/Ideal P Gain Fdbk/Disabled'
//  '<S170>' : 'controller/x PID/Integrator/Disabled'
//  '<S171>' : 'controller/x PID/Integrator ICs/Disabled'
//  '<S172>' : 'controller/x PID/N Copy/Disabled'
//  '<S173>' : 'controller/x PID/N Gain/Internal Parameters'
//  '<S174>' : 'controller/x PID/P Copy/Disabled'
//  '<S175>' : 'controller/x PID/Parallel P Gain/Internal Parameters'
//  '<S176>' : 'controller/x PID/Reset Signal/Disabled'
//  '<S177>' : 'controller/x PID/Saturation/Passthrough'
//  '<S178>' : 'controller/x PID/Saturation Fdbk/Disabled'
//  '<S179>' : 'controller/x PID/Sum/Sum_PD'
//  '<S180>' : 'controller/x PID/Sum Fdbk/Disabled'
//  '<S181>' : 'controller/x PID/Tracking Mode/Disabled'
//  '<S182>' : 'controller/x PID/Tracking Mode Sum/Passthrough'
//  '<S183>' : 'controller/x PID/postSat Signal/Forward_Path'
//  '<S184>' : 'controller/x PID/preSat Signal/Forward_Path'
//  '<S185>' : 'controller/y PID/Anti-windup'
//  '<S186>' : 'controller/y PID/D Gain'
//  '<S187>' : 'controller/y PID/Filter'
//  '<S188>' : 'controller/y PID/Filter ICs'
//  '<S189>' : 'controller/y PID/I Gain'
//  '<S190>' : 'controller/y PID/Ideal P Gain'
//  '<S191>' : 'controller/y PID/Ideal P Gain Fdbk'
//  '<S192>' : 'controller/y PID/Integrator'
//  '<S193>' : 'controller/y PID/Integrator ICs'
//  '<S194>' : 'controller/y PID/N Copy'
//  '<S195>' : 'controller/y PID/N Gain'
//  '<S196>' : 'controller/y PID/P Copy'
//  '<S197>' : 'controller/y PID/Parallel P Gain'
//  '<S198>' : 'controller/y PID/Reset Signal'
//  '<S199>' : 'controller/y PID/Saturation'
//  '<S200>' : 'controller/y PID/Saturation Fdbk'
//  '<S201>' : 'controller/y PID/Sum'
//  '<S202>' : 'controller/y PID/Sum Fdbk'
//  '<S203>' : 'controller/y PID/Tracking Mode'
//  '<S204>' : 'controller/y PID/Tracking Mode Sum'
//  '<S205>' : 'controller/y PID/postSat Signal'
//  '<S206>' : 'controller/y PID/preSat Signal'
//  '<S207>' : 'controller/y PID/Anti-windup/Disabled'
//  '<S208>' : 'controller/y PID/D Gain/Internal Parameters'
//  '<S209>' : 'controller/y PID/Filter/Disc. Forward Euler Filter'
//  '<S210>' : 'controller/y PID/Filter ICs/Internal IC - Filter'
//  '<S211>' : 'controller/y PID/I Gain/Disabled'
//  '<S212>' : 'controller/y PID/Ideal P Gain/Passthrough'
//  '<S213>' : 'controller/y PID/Ideal P Gain Fdbk/Disabled'
//  '<S214>' : 'controller/y PID/Integrator/Disabled'
//  '<S215>' : 'controller/y PID/Integrator ICs/Disabled'
//  '<S216>' : 'controller/y PID/N Copy/Disabled'
//  '<S217>' : 'controller/y PID/N Gain/Internal Parameters'
//  '<S218>' : 'controller/y PID/P Copy/Disabled'
//  '<S219>' : 'controller/y PID/Parallel P Gain/Internal Parameters'
//  '<S220>' : 'controller/y PID/Reset Signal/Disabled'
//  '<S221>' : 'controller/y PID/Saturation/Passthrough'
//  '<S222>' : 'controller/y PID/Saturation Fdbk/Disabled'
//  '<S223>' : 'controller/y PID/Sum/Sum_PD'
//  '<S224>' : 'controller/y PID/Sum Fdbk/Disabled'
//  '<S225>' : 'controller/y PID/Tracking Mode/Disabled'
//  '<S226>' : 'controller/y PID/Tracking Mode Sum/Passthrough'
//  '<S227>' : 'controller/y PID/postSat Signal/Forward_Path'
//  '<S228>' : 'controller/y PID/preSat Signal/Forward_Path'
//  '<S229>' : 'controller/z PID /Anti-windup'
//  '<S230>' : 'controller/z PID /D Gain'
//  '<S231>' : 'controller/z PID /Filter'
//  '<S232>' : 'controller/z PID /Filter ICs'
//  '<S233>' : 'controller/z PID /I Gain'
//  '<S234>' : 'controller/z PID /Ideal P Gain'
//  '<S235>' : 'controller/z PID /Ideal P Gain Fdbk'
//  '<S236>' : 'controller/z PID /Integrator'
//  '<S237>' : 'controller/z PID /Integrator ICs'
//  '<S238>' : 'controller/z PID /N Copy'
//  '<S239>' : 'controller/z PID /N Gain'
//  '<S240>' : 'controller/z PID /P Copy'
//  '<S241>' : 'controller/z PID /Parallel P Gain'
//  '<S242>' : 'controller/z PID /Reset Signal'
//  '<S243>' : 'controller/z PID /Saturation'
//  '<S244>' : 'controller/z PID /Saturation Fdbk'
//  '<S245>' : 'controller/z PID /Sum'
//  '<S246>' : 'controller/z PID /Sum Fdbk'
//  '<S247>' : 'controller/z PID /Tracking Mode'
//  '<S248>' : 'controller/z PID /Tracking Mode Sum'
//  '<S249>' : 'controller/z PID /postSat Signal'
//  '<S250>' : 'controller/z PID /preSat Signal'
//  '<S251>' : 'controller/z PID /Anti-windup/Passthrough'
//  '<S252>' : 'controller/z PID /D Gain/Internal Parameters'
//  '<S253>' : 'controller/z PID /Filter/Disc. Forward Euler Filter'
//  '<S254>' : 'controller/z PID /Filter ICs/Internal IC - Filter'
//  '<S255>' : 'controller/z PID /I Gain/Internal Parameters'
//  '<S256>' : 'controller/z PID /Ideal P Gain/Passthrough'
//  '<S257>' : 'controller/z PID /Ideal P Gain Fdbk/Disabled'
//  '<S258>' : 'controller/z PID /Integrator/Discrete'
//  '<S259>' : 'controller/z PID /Integrator ICs/Internal IC'
//  '<S260>' : 'controller/z PID /N Copy/Disabled'
//  '<S261>' : 'controller/z PID /N Gain/Internal Parameters'
//  '<S262>' : 'controller/z PID /P Copy/Disabled'
//  '<S263>' : 'controller/z PID /Parallel P Gain/Internal Parameters'
//  '<S264>' : 'controller/z PID /Reset Signal/Disabled'
//  '<S265>' : 'controller/z PID /Saturation/Passthrough'
//  '<S266>' : 'controller/z PID /Saturation Fdbk/Disabled'
//  '<S267>' : 'controller/z PID /Sum/Sum_PID'
//  '<S268>' : 'controller/z PID /Sum Fdbk/Disabled'
//  '<S269>' : 'controller/z PID /Tracking Mode/Disabled'
//  '<S270>' : 'controller/z PID /Tracking Mode Sum/Passthrough'
//  '<S271>' : 'controller/z PID /postSat Signal/Forward_Path'
//  '<S272>' : 'controller/z PID /preSat Signal/Forward_Path'

#endif                                 // RTW_HEADER_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
