//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planning_fun.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 22-May-2021 11:59:19
//
#ifndef PLANNING_FUN_H
#define PLANNING_FUN_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void planning_fun(const double state_robot[6], double dt, const double
  limit[2], const double goal[2], const coder::array<unsigned char, 2U> &image,
  double resolution, double maxIter, coder::array<double, 2U> &final_path);

#endif

//
// File trailer for planning_fun.h
//
// [EOF]
//
