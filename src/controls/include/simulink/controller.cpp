//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: controller.cpp
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
#include "controller.h"
#include "invNxN_YgskRBgH.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
# define rtmGetErrorStatusPointer(rtm) (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
# define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

// Output and update for referenced model: 'controller'
void controllerModelClass::step(const real_T arg_eta_d[6], const real_T
  arg_eta_d_dot_dot[6], const real_T arg_x[12], real_T arg_tau_[6])
{
  // local block i/o variables
  real_T rtb_eta_dot[6];
  real_T nu[6];
  real_T rtb_eta_err[6];
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_df;
  real_T rtb_FilterCoefficient_n;
  real_T rtb_FilterCoefficient_g;
  real_T rtb_FilterCoefficient_l;
  real_T rtb_FilterCoefficient_ad;
  real_T sth_tmp;
  real_T cth_tmp;
  real_T spsi_tmp;
  real_T cpsi_tmp;
  real_T sphi_tmp;
  real_T cphi_tmp;
  int32_T i;
  real_T cpsi_tmp_0[36];
  real_T tmp[36];
  real_T tmp_0[36];
  real_T arg_x_0[6];
  real_T rtb_Sum_0[6];
  real_T tmp_1[6];
  real_T tmp_2[36];
  real_T tmp_3[36];
  real_T tmp_4[6];
  int32_T i_0;
  real_T cpsi_tmp_tmp;
  real_T cpsi_tmp_tmp_0;
  int32_T cpsi_tmp_tmp_1;
  real_T tmp_5;
  int32_T tmp_6;

  // MATLAB Function: '<Root>/C(nu)*nu + D(nu)*nu + g(eta)'
  nu[0] = arg_x[0];
  nu[1] = arg_x[1];
  nu[2] = arg_x[2];
  nu[3] = arg_x[3];
  nu[4] = arg_x[4];
  nu[5] = arg_x[5];

  // Sum: '<Root>/Sum'
  for (i = 0; i < 6; i++) {
    rtb_eta_err[i] = arg_eta_d[i] - arg_x[i + 6];
  }

  // End of Sum: '<Root>/Sum'

  // Gain: '<S173>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S165>/Filter'
  //   Gain: '<S164>/Derivative Gain'
  //   Sum: '<S165>/SumD'

  rtb_FilterCoefficient = (19.9620628749571 * rtb_eta_err[0] -
    controller_DW.Filter_DSTATE) * 2285.6559520832;

  // Gain: '<S217>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S209>/Filter'
  //   Gain: '<S208>/Derivative Gain'
  //   Sum: '<S209>/SumD'

  rtb_FilterCoefficient_df = (19.9793271274214 * rtb_eta_err[1] -
    controller_DW.Filter_DSTATE_k) * 2285.6559520832;

  // Gain: '<S261>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S253>/Filter'
  //   Gain: '<S252>/Derivative Gain'
  //   Sum: '<S253>/SumD'

  rtb_FilterCoefficient_n = (19.1379786709817 * rtb_eta_err[2] -
    controller_DW.Filter_DSTATE_i) * 54.1293789257892;

  // Gain: '<S41>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S33>/Filter'
  //   Gain: '<S32>/Derivative Gain'
  //   Sum: '<S33>/SumD'

  rtb_FilterCoefficient_g = (19.9398731418498 * rtb_eta_err[3] -
    controller_DW.Filter_DSTATE_d) * 2285.6559520832;

  // Gain: '<S129>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S121>/Filter'
  //   Gain: '<S120>/Derivative Gain'
  //   Sum: '<S121>/SumD'

  rtb_FilterCoefficient_l = (19.9254194428844 * rtb_eta_err[4] -
    controller_DW.Filter_DSTATE_a) * 2285.6559520832;

  // Gain: '<S85>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S77>/Filter'
  //   Gain: '<S76>/Derivative Gain'
  //   Sum: '<S77>/SumD'

  rtb_FilterCoefficient_ad = (19.9254194628397 * rtb_eta_err[5] -
    controller_DW.Filter_DSTATE_l) * 2285.6559520832;

  // ModelReference: '<Root>/Model'
  ModelMDLOBJ1.step(&arg_x[0], &arg_x[6], &rtb_eta_dot[0]);

  // MATLAB Function: '<Root>/M*J^-1(eta)*(a_n - J_dot(eta, eta_dot)*nu)' incorporates:
  //   MATLAB Function: '<Root>/C(nu)*nu + D(nu)*nu + g(eta)'

  cphi_tmp = std::cos(arg_x[9]);
  sphi_tmp = std::sin(arg_x[9]);
  cth_tmp = std::cos(arg_x[10]);
  sth_tmp = std::sin(arg_x[10]);
  cpsi_tmp = std::cos(arg_x[11]);
  spsi_tmp = std::sin(arg_x[11]);
  cpsi_tmp_0[0] = cpsi_tmp * cth_tmp;
  cpsi_tmp_0[6] = cpsi_tmp * sth_tmp * sphi_tmp + -spsi_tmp * cphi_tmp;
  cpsi_tmp_tmp = cpsi_tmp * cphi_tmp;
  cpsi_tmp_0[12] = cpsi_tmp_tmp * sth_tmp + spsi_tmp * sphi_tmp;
  cpsi_tmp_0[1] = spsi_tmp * cth_tmp;
  cpsi_tmp_tmp_0 = sphi_tmp * sth_tmp;
  cpsi_tmp_0[7] = cpsi_tmp_tmp_0 * spsi_tmp + cpsi_tmp_tmp;
  cpsi_tmp_0[13] = sth_tmp * spsi_tmp * cphi_tmp + -cpsi_tmp * sphi_tmp;
  cpsi_tmp_0[2] = -sth_tmp;
  cpsi_tmp_0[8] = cth_tmp * sphi_tmp;
  cpsi_tmp_0[14] = cth_tmp * cphi_tmp;
  for (i = 0; i < 3; i++) {
    cpsi_tmp_tmp_1 = 6 * (i + 3);
    cpsi_tmp_0[cpsi_tmp_tmp_1] = 0.0;
    cpsi_tmp_0[6 * i + 3] = 0.0;
    cpsi_tmp_0[cpsi_tmp_tmp_1 + 1] = 0.0;
    cpsi_tmp_0[6 * i + 4] = 0.0;
    cpsi_tmp_0[cpsi_tmp_tmp_1 + 2] = 0.0;
    cpsi_tmp_0[6 * i + 5] = 0.0;
  }

  cpsi_tmp_0[21] = 1.0;
  cpsi_tmp_0[27] = cpsi_tmp_tmp_0 / cth_tmp;
  cpsi_tmp_0[33] = cphi_tmp * sth_tmp / cth_tmp;
  cpsi_tmp_0[22] = 0.0;
  cpsi_tmp_0[28] = cphi_tmp;
  cpsi_tmp_0[34] = -sphi_tmp;
  cpsi_tmp_0[23] = 0.0;
  cpsi_tmp_0[29] = sphi_tmp / cth_tmp;
  cpsi_tmp_0[35] = cphi_tmp / cth_tmp;
  invNxN_YgskRBgH(cpsi_tmp_0, tmp);
  cpsi_tmp_0[0] = 18.7;
  cpsi_tmp_0[6] = 0.0;
  cpsi_tmp_0[12] = 0.0;
  cpsi_tmp_0[18] = 0.0;
  cpsi_tmp_0[24] = 0.0;
  cpsi_tmp_0[30] = 0.0;
  cpsi_tmp_0[1] = 0.0;
  cpsi_tmp_0[7] = 18.7;
  cpsi_tmp_0[13] = 0.0;
  cpsi_tmp_0[19] = 0.0;
  cpsi_tmp_0[25] = 0.0;
  cpsi_tmp_0[31] = 0.0;
  cpsi_tmp_0[2] = 0.0;
  cpsi_tmp_0[8] = 0.0;
  cpsi_tmp_0[14] = 18.7;
  cpsi_tmp_0[20] = 0.0;
  cpsi_tmp_0[26] = 0.0;
  cpsi_tmp_0[32] = 0.0;
  cpsi_tmp_0[3] = 0.0;
  cpsi_tmp_0[9] = 0.0;
  cpsi_tmp_0[15] = 0.0;
  cpsi_tmp_0[21] = 13.4;
  cpsi_tmp_0[27] = 0.0;
  cpsi_tmp_0[33] = 0.0;
  cpsi_tmp_0[4] = 0.0;
  cpsi_tmp_0[10] = 0.0;
  cpsi_tmp_0[16] = 0.0;
  cpsi_tmp_0[22] = 0.0;
  cpsi_tmp_0[28] = 15.7;
  cpsi_tmp_0[34] = 0.0;
  cpsi_tmp_0[5] = 0.0;
  cpsi_tmp_0[11] = 0.0;
  cpsi_tmp_0[17] = 0.0;
  cpsi_tmp_0[23] = 0.0;
  cpsi_tmp_0[29] = 0.0;
  cpsi_tmp_0[35] = 8.98;
  tmp_0[0] = -rtb_eta_dot[5] * cth_tmp * spsi_tmp - rtb_eta_dot[4] * cpsi_tmp *
    sth_tmp;
  tmp_0[6] = (((rtb_eta_dot[3] * sphi_tmp * spsi_tmp - rtb_eta_dot[5] * cphi_tmp
                * cpsi_tmp) + rtb_eta_dot[3] * cphi_tmp * cpsi_tmp * sth_tmp) +
              rtb_eta_dot[4] * std::cos(arg_x[11]) * cth_tmp * sphi_tmp) -
    rtb_eta_dot[5] * sphi_tmp * spsi_tmp * sth_tmp;
  cpsi_tmp_tmp = rtb_eta_dot[3] * std::cos(arg_x[9]);
  tmp_0[12] = (((cpsi_tmp_tmp * spsi_tmp + rtb_eta_dot[5] * cpsi_tmp * sphi_tmp)
                + rtb_eta_dot[4] * cphi_tmp * cpsi_tmp * cth_tmp) - rtb_eta_dot
               [3] * cpsi_tmp * sphi_tmp * sth_tmp) - rtb_eta_dot[5] * std::cos
    (arg_x[9]) * spsi_tmp * sth_tmp;
  tmp_0[18] = 0.0;
  tmp_0[24] = 0.0;
  tmp_0[30] = 0.0;
  tmp_0[1] = rtb_eta_dot[5] * std::cos(arg_x[11]) * cth_tmp - rtb_eta_dot[4] *
    spsi_tmp * sth_tmp;
  tmp_0[7] = (((rtb_eta_dot[3] * std::cos(arg_x[9]) * std::sin(arg_x[11]) *
                sth_tmp - rtb_eta_dot[5] * std::cos(arg_x[9]) * std::sin(arg_x
    [11])) - rtb_eta_dot[3] * std::cos(arg_x[11]) * std::sin(arg_x[9])) +
              rtb_eta_dot[5] * std::cos(arg_x[11]) * std::sin(arg_x[9]) *
              sth_tmp) + rtb_eta_dot[4] * cth_tmp * sphi_tmp * spsi_tmp;
  cpsi_tmp = rtb_eta_dot[4] * std::cos(arg_x[9]);
  tmp_0[13] = (((rtb_eta_dot[5] * std::sin(arg_x[9]) * std::sin(arg_x[11]) -
                 rtb_eta_dot[3] * std::cos(arg_x[9]) * std::cos(arg_x[11])) +
                rtb_eta_dot[5] * std::cos(arg_x[9]) * std::cos(arg_x[11]) *
                sth_tmp) + cpsi_tmp * cth_tmp * spsi_tmp) - rtb_eta_dot[3] * std::
    sin(arg_x[9]) * std::sin(arg_x[11]) * sth_tmp;
  tmp_0[19] = 0.0;
  tmp_0[25] = 0.0;
  tmp_0[31] = 0.0;
  tmp_0[2] = -rtb_eta_dot[4] * cth_tmp;
  tmp_0[8] = cpsi_tmp_tmp * cth_tmp - rtb_eta_dot[4] * sphi_tmp * sth_tmp;
  tmp_0[14] = -rtb_eta_dot[3] * cth_tmp * sphi_tmp - cpsi_tmp * sth_tmp;
  tmp_0[20] = 0.0;
  tmp_0[26] = 0.0;
  tmp_0[32] = 0.0;
  tmp_0[3] = 0.0;
  tmp_0[9] = 0.0;
  tmp_0[15] = 0.0;
  tmp_0[21] = 0.0;
  spsi_tmp = rtb_eta_dot[4] * std::sin(arg_x[9]);
  cpsi_tmp_tmp_0 = sth_tmp * sth_tmp;
  tmp_5 = cth_tmp * cth_tmp;
  tmp_0[27] = (spsi_tmp * cpsi_tmp_tmp_0 / tmp_5 + spsi_tmp) + cpsi_tmp_tmp *
    sth_tmp / cth_tmp;
  spsi_tmp = rtb_eta_dot[3] * std::sin(arg_x[9]);
  tmp_0[33] = (cpsi_tmp * cpsi_tmp_tmp_0 / tmp_5 + cpsi_tmp) - spsi_tmp *
    sth_tmp / cth_tmp;
  tmp_0[4] = 0.0;
  tmp_0[10] = 0.0;
  tmp_0[16] = 0.0;
  tmp_0[22] = 0.0;
  tmp_0[28] = -rtb_eta_dot[3] * sphi_tmp;
  tmp_0[34] = -rtb_eta_dot[3] * cphi_tmp;
  tmp_0[5] = 0.0;
  tmp_0[11] = 0.0;
  tmp_0[17] = 0.0;
  tmp_0[23] = 0.0;
  tmp_0[29] = rtb_eta_dot[4] * std::sin(arg_x[9]) * std::sin(arg_x[10]) / tmp_5
    + cpsi_tmp_tmp / cth_tmp;
  tmp_0[35] = rtb_eta_dot[4] * std::cos(arg_x[9]) * std::sin(arg_x[10]) / tmp_5
    - spsi_tmp / cth_tmp;
  arg_x_0[0] = arg_x[0];
  arg_x_0[1] = arg_x[1];
  arg_x_0[2] = arg_x[2];
  arg_x_0[3] = arg_x[3];
  arg_x_0[4] = arg_x[4];
  arg_x_0[5] = arg_x[5];

  // Sum: '<Root>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S258>/Integrator'
  //   Gain: '<S131>/Proportional Gain'
  //   Gain: '<S175>/Proportional Gain'
  //   Gain: '<S219>/Proportional Gain'
  //   Gain: '<S263>/Proportional Gain'
  //   Gain: '<S43>/Proportional Gain'
  //   Gain: '<S87>/Proportional Gain'
  //   Sum: '<S135>/Sum'
  //   Sum: '<S179>/Sum'
  //   Sum: '<S223>/Sum'
  //   Sum: '<S267>/Sum'
  //   Sum: '<S47>/Sum'
  //   Sum: '<S91>/Sum'

  rtb_Sum_0[0] = (21.6230251127914 * rtb_eta_err[0] + rtb_FilterCoefficient) +
    arg_eta_d_dot_dot[0];
  rtb_Sum_0[1] = (15.3463764949904 * rtb_eta_err[1] + rtb_FilterCoefficient_df)
    + arg_eta_d_dot_dot[1];
  rtb_Sum_0[2] = ((100.423645619639 * rtb_eta_err[2] +
                   controller_DW.Integrator_DSTATE) + rtb_FilterCoefficient_n) +
    arg_eta_d_dot_dot[2];
  rtb_Sum_0[3] = (27.8944548552175 * rtb_eta_err[3] + rtb_FilterCoefficient_g) +
    arg_eta_d_dot_dot[3];
  rtb_Sum_0[4] = (31.3755535919802 * rtb_eta_err[4] + rtb_FilterCoefficient_l) +
    arg_eta_d_dot_dot[4];
  rtb_Sum_0[5] = (31.3755266700164 * rtb_eta_err[5] + rtb_FilterCoefficient_ad)
    + arg_eta_d_dot_dot[5];

  // MATLAB Function: '<Root>/M*J^-1(eta)*(a_n - J_dot(eta, eta_dot)*nu)'
  for (i = 0; i < 6; i++) {
    tmp_1[i] = 0.0;
    for (cpsi_tmp_tmp_1 = 0; cpsi_tmp_tmp_1 < 6; cpsi_tmp_tmp_1++) {
      tmp_1[i] += tmp_0[6 * cpsi_tmp_tmp_1 + i] * arg_x_0[cpsi_tmp_tmp_1];
    }
  }

  for (i = 0; i < 6; i++) {
    for (cpsi_tmp_tmp_1 = 0; cpsi_tmp_tmp_1 < 6; cpsi_tmp_tmp_1++) {
      tmp_6 = i + 6 * cpsi_tmp_tmp_1;
      tmp_0[tmp_6] = 0.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        tmp_0[tmp_6] += cpsi_tmp_0[6 * i_0 + i] * tmp[6 * cpsi_tmp_tmp_1 + i_0];
      }
    }

    arg_x_0[i] = rtb_Sum_0[i] - tmp_1[i];
  }

  // MATLAB Function: '<Root>/C(nu)*nu + D(nu)*nu + g(eta)'
  tmp[0] = 0.0;
  tmp[6] = 0.0;
  tmp[12] = 0.0;
  tmp[18] = 0.0;
  tmp[24] = 18.7 * arg_x[2];
  tmp[30] = -18.7 * arg_x[2];
  tmp[1] = 0.0;
  tmp[7] = 0.0;
  tmp[13] = 0.0;
  tmp[19] = -18.7 * arg_x[2];
  tmp[25] = 0.0;
  tmp[31] = 18.7 * arg_x[0];
  tmp[2] = 0.0;
  tmp[8] = 0.0;
  tmp[14] = 0.0;
  tmp[20] = 18.7 * arg_x[1];
  tmp[26] = -18.7 * arg_x[0];
  tmp[32] = 0.0;
  tmp[3] = 0.0;
  tmp[9] = 18.7 * arg_x[2];
  tmp[15] = -18.7 * arg_x[1];
  tmp[21] = 0.0;
  tmp[27] = 8.98 * arg_x[5];
  tmp[33] = -15.7 * arg_x[4];
  tmp[4] = -18.7 * arg_x[2];
  tmp[10] = 0.0;
  tmp[16] = 18.7 * arg_x[0];
  tmp[22] = -8.98 * arg_x[5];
  tmp[28] = 0.0;
  tmp[34] = 13.4 * arg_x[3];
  tmp[5] = 18.7 * arg_x[1];
  tmp[11] = -18.7 * arg_x[0];
  tmp[17] = 0.0;
  tmp[23] = 15.7 * arg_x[4];
  tmp[29] = -13.4 * arg_x[3];
  tmp[35] = 0.0;
  cpsi_tmp_0[0] = 0.525;
  cpsi_tmp_0[6] = 0.0;
  cpsi_tmp_0[12] = 0.0;
  cpsi_tmp_0[18] = 0.0;
  cpsi_tmp_0[24] = 0.0;
  cpsi_tmp_0[30] = 0.0;
  cpsi_tmp_0[1] = 0.0;
  cpsi_tmp_0[7] = 0.0;
  cpsi_tmp_0[13] = 0.0;
  cpsi_tmp_0[19] = 0.0;
  cpsi_tmp_0[25] = 0.0;
  cpsi_tmp_0[31] = 0.0;
  cpsi_tmp_0[2] = 0.0;
  cpsi_tmp_0[8] = 0.0;
  cpsi_tmp_0[14] = 7.25;
  cpsi_tmp_0[20] = 0.0;
  cpsi_tmp_0[26] = 0.0;
  cpsi_tmp_0[32] = 0.0;
  cpsi_tmp_0[3] = 0.0;
  cpsi_tmp_0[9] = 0.0;
  cpsi_tmp_0[15] = 0.0;
  cpsi_tmp_0[21] = 0.0;
  cpsi_tmp_0[27] = 0.0;
  cpsi_tmp_0[33] = 0.0;
  cpsi_tmp_0[4] = 0.0;
  cpsi_tmp_0[10] = 0.0;
  cpsi_tmp_0[16] = 0.0;
  cpsi_tmp_0[22] = 0.0;
  cpsi_tmp_0[28] = 0.0;
  cpsi_tmp_0[34] = 0.0;
  cpsi_tmp_0[5] = 0.0;
  cpsi_tmp_0[11] = 0.0;
  cpsi_tmp_0[17] = 0.0;
  cpsi_tmp_0[23] = 0.0;
  cpsi_tmp_0[29] = 0.0;
  cpsi_tmp_0[35] = 0.0;
  tmp_2[0] = 85.625 * std::abs(arg_x[0]);
  tmp_2[6] = 0.0;
  tmp_2[12] = 0.0;
  tmp_2[18] = 0.0;
  tmp_2[24] = 0.0;
  tmp_2[30] = 0.0;
  tmp_2[1] = 0.0;
  tmp_2[7] = 0.0 * std::abs(arg_x[1]);
  tmp_2[13] = 0.0;
  tmp_2[19] = 0.0;
  tmp_2[25] = 0.0;
  tmp_2[31] = 0.0;
  tmp_2[2] = 0.0;
  tmp_2[8] = 0.0;
  tmp_2[14] = 215.0 * std::abs(arg_x[2]);
  tmp_2[20] = 0.0;
  tmp_2[26] = 0.0;
  tmp_2[32] = 0.0;
  tmp_2[3] = 0.0;
  tmp_2[9] = 0.0;
  tmp_2[15] = 0.0;
  tmp_2[21] = 0.0 * std::abs(arg_x[3]);
  tmp_2[27] = 0.0;
  tmp_2[33] = 0.0;
  tmp_2[4] = 0.0;
  tmp_2[10] = 0.0;
  tmp_2[16] = 0.0;
  tmp_2[22] = 0.0;
  tmp_2[28] = 0.0 * std::abs(arg_x[4]);
  tmp_2[34] = 0.0;
  tmp_2[5] = 0.0;
  tmp_2[11] = 0.0;
  tmp_2[17] = 0.0;
  tmp_2[23] = 0.0;
  tmp_2[29] = 0.0;
  tmp_2[35] = 0.0 * std::abs(arg_x[5]);
  for (i = 0; i < 36; i++) {
    tmp_3[i] = cpsi_tmp_0[i] + tmp_2[i];
  }

  tmp_4[0] = -0.61214400000002911 * sth_tmp;
  tmp_4[1] = 0.61214400000002911 * cth_tmp * sphi_tmp;
  tmp_4[2] = 0.61214400000002911 * std::cos(arg_x[10]) * cphi_tmp;
  tmp_4[3] = -0.0 * cth_tmp * cphi_tmp + -18.405914400000004 * cth_tmp *
    sphi_tmp;
  tmp_4[4] = 0.0 * cth_tmp * cphi_tmp + -18.405914400000004 * sth_tmp;
  tmp_4[5] = -0.0 * cth_tmp * sphi_tmp - 0.0 * sth_tmp;
  for (i = 0; i < 6; i++) {
    tmp_1[i] = 0.0;
    rtb_Sum_0[i] = 0.0;

    // Sum: '<Root>/Sum4' incorporates:
    //   MATLAB Function: '<Root>/M*J^-1(eta)*(a_n - J_dot(eta, eta_dot)*nu)'

    cpsi_tmp_tmp = 0.0;
    for (cpsi_tmp_tmp_1 = 0; cpsi_tmp_tmp_1 < 6; cpsi_tmp_tmp_1++) {
      tmp_6 = 6 * cpsi_tmp_tmp_1 + i;
      cpsi_tmp_tmp += tmp_0[tmp_6] * arg_x_0[cpsi_tmp_tmp_1];
      tmp_1[i] += tmp[tmp_6] * nu[cpsi_tmp_tmp_1];
      rtb_Sum_0[i] += tmp_3[tmp_6] * nu[cpsi_tmp_tmp_1];
    }

    arg_tau_[i] = ((tmp_1[i] + rtb_Sum_0[i]) + tmp_4[i]) + cpsi_tmp_tmp;

    // End of Sum: '<Root>/Sum4'
  }

  // Update for DiscreteIntegrator: '<S165>/Filter'
  controller_DW.Filter_DSTATE += 0.2 * rtb_FilterCoefficient;

  // Update for DiscreteIntegrator: '<S209>/Filter'
  controller_DW.Filter_DSTATE_k += 0.2 * rtb_FilterCoefficient_df;

  // Update for DiscreteIntegrator: '<S258>/Integrator' incorporates:
  //   Gain: '<S255>/Integral Gain'

  controller_DW.Integrator_DSTATE += 119.359603608016 * rtb_eta_err[2] * 0.2;

  // Update for DiscreteIntegrator: '<S253>/Filter'
  controller_DW.Filter_DSTATE_i += 0.2 * rtb_FilterCoefficient_n;

  // Update for DiscreteIntegrator: '<S33>/Filter'
  controller_DW.Filter_DSTATE_d += 0.2 * rtb_FilterCoefficient_g;

  // Update for DiscreteIntegrator: '<S121>/Filter'
  controller_DW.Filter_DSTATE_a += 0.2 * rtb_FilterCoefficient_l;

  // Update for DiscreteIntegrator: '<S77>/Filter'
  controller_DW.Filter_DSTATE_l += 0.2 * rtb_FilterCoefficient_ad;
}

// Model initialize function
void controllerModelClass::initialize()
{
  // Model Initialize function for ModelReference Block: '<Root>/Model'

  // Set error status pointer for ModelReference Block: '<Root>/Model'
  ModelMDLOBJ1.setErrorStatusPointer(rtmGetErrorStatusPointer((&controller_M)));
}

// Constructor
controllerModelClass::controllerModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
controllerModelClass::~controllerModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_controller_T * controllerModelClass::getRTM()
{
  return (&controller_M);
}

// member function to setup error status pointer
void controllerModelClass::setErrorStatusPointer(const char_T **rt_errorStatus)
{
  rtmSetErrorStatusPointer((&controller_M), rt_errorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//
