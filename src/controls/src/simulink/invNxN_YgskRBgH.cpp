//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: invNxN_YgskRBgH.cpp
//
// Code generated for Simulink model 'controller'.
//
// Model version                  : 1.72
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Fri Mar 13 18:49:31 2020
//
#include "rtwtypes.h"
#include <cmath>
#include "invNxN_YgskRBgH.h"

// Function for MATLAB Function: '<Root>/M*J^-1(eta)*(a_n - J_dot(eta, eta_dot)*nu)'
void invNxN_YgskRBgH(const real_T x[36], real_T y[36])
{
  int8_T p[6];
  real_T A[36];
  int8_T ipiv[6];
  int32_T pipk;
  int32_T b_j;
  int32_T ix;
  real_T smax;
  int32_T d_k;
  int32_T iy;
  real_T b_y;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  for (d_k = 0; d_k < 36; d_k++) {
    y[d_k] = 0.0;
    A[d_k] = x[d_k];
  }

  for (d_k = 0; d_k < 6; d_k++) {
    ipiv[d_k] = static_cast<int8_T>((d_k + 1));
  }

  for (b_j = 0; b_j < 5; b_j++) {
    pipk = b_j * 7;
    iy = 0;
    ix = pipk;
    smax = std::abs(A[pipk]);
    for (d_k = 2; d_k <= 6 - b_j; d_k++) {
      ix++;
      b_y = std::abs(A[ix]);
      if (b_y > smax) {
        iy = d_k - 1;
        smax = b_y;
      }
    }

    if (A[pipk + iy] != 0.0) {
      if (iy != 0) {
        iy += b_j;
        ipiv[b_j] = static_cast<int8_T>((iy + 1));
        ix = b_j;
        for (d_k = 0; d_k < 6; d_k++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      iy = (pipk - b_j) + 6;
      for (ix = pipk + 1; ix < iy; ix++) {
        A[ix] /= A[pipk];
      }
    }

    iy = pipk;
    ix = pipk + 6;
    for (d_k = 0; d_k <= 4 - b_j; d_k++) {
      if (A[ix] != 0.0) {
        smax = -A[ix];
        c_ix = pipk + 1;
        d = (iy - b_j) + 12;
        for (ijA = iy + 7; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * smax;
          c_ix++;
        }
      }

      ix += 6;
      iy += 6;
    }
  }

  for (d_k = 0; d_k < 6; d_k++) {
    p[d_k] = static_cast<int8_T>((d_k + 1));
  }

  for (b_j = 0; b_j < 5; b_j++) {
    if (ipiv[b_j] > b_j + 1) {
      d = ipiv[b_j] - 1;
      pipk = p[d];
      p[d] = p[b_j];
      p[b_j] = static_cast<int8_T>(pipk);
    }
  }

  for (b_j = 0; b_j < 6; b_j++) {
    d = p[b_j] - 1;
    y[b_j + 6 * d] = 1.0;
    for (iy = b_j; iy + 1 < 7; iy++) {
      d_k = 6 * d + iy;
      if (y[d_k] != 0.0) {
        for (ix = iy + 1; ix + 1 < 7; ix++) {
          c_ix = 6 * d + ix;
          y[c_ix] -= y[d_k] * A[6 * iy + ix];
        }
      }
    }
  }

  for (b_j = 0; b_j < 6; b_j++) {
    pipk = 6 * b_j;
    for (iy = 5; iy >= 0; iy--) {
      ix = 6 * iy;
      d_k = iy + pipk;
      if (y[d_k] != 0.0) {
        y[d_k] /= A[iy + ix];
        for (d_k = 0; d_k < iy; d_k++) {
          c_ix = d_k + pipk;
          y[c_ix] -= y[iy + pipk] * A[d_k + ix];
        }
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
