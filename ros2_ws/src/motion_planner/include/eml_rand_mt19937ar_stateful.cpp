//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 22-May-2021 11:59:19
//

// Include Files
#include "eml_rand_mt19937ar_stateful.h"
#include "planning_fun_data.h"
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void eml_rand_mt19937ar_stateful_init()
{
  unsigned int r;

  //@>2a7d9
  std::memset(&                        //@>28eef
              state                    //@>28ef0
              [                        //@>28eec
              0],                      //@>37f0c
              0,                       //@>28eee
              625U                     //@>28ef6
              *                        //@>28ef5
              sizeof(unsigned int));

  //@>18a8a
  r                                    //@>18a89
    =                                  //@>195e5
    5489U;

  //@>1deee
  state                                //@>18ab6
    [                                  //@>18ac7
    0]                                 //@>18a8d
    =                                  //@>195e6
    5489U;

  //@>18ac0
  for (int                             //@>37f11
       mti =                           //@>18aa8
       0;                              //@>37f19
       mti <                           //@>18aa6
       623;                            //@>37f16
       mti                             //@>37f1d
       ++) {
    //@>18a92
    r                                  //@>18a91
      = ((                             //@>18a93
          r                            //@>231c0
          ^                            //@>18a95
          r                            //@>18ab3
          >>                           //@>18ac8
          30U)                         //@>231bd
         *                             //@>18ac9
         1812433253U                   //@>23398
         +                             //@>1a936
         mti)                          //@>23399
      +                                //@>18a99
      1U;

    //@>1defe
    state                              //@>18ab7
      [                                //@>1a93f
      mti                              //@>1aa92
      +                                //@>18ab9
      1]                               //@>18a9a
      =                                //@>18a9e
      r;
  }

  //@>1def4
  state                                //@>18abb
    [                                  //@>18aca
    624]                               //@>18aa0
    =                                  //@>18acb
    624U;
}

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
