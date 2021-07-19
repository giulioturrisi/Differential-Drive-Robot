//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planning_fun_initialize.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 19-Jul-2021 23:22:17
//

// Include Files
#include "planning_fun_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "planning_fun_data.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void planning_fun_initialize()
{
  eml_rand_mt19937ar_stateful_init();
  isInitialized_planning_fun = true;
}

//
// File trailer for planning_fun_initialize.cpp
//
// [EOF]
//
