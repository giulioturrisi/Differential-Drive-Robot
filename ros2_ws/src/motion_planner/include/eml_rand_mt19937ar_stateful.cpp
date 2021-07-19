//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 19-Jul-2021 23:22:17
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

  //@>2a05c
  std::memset(&                        //@>286e2
              state                    //@>286e3
              [                        //@>286df
              0],                      //@>3834b
              0,                       //@>286e1
              625U                     //@>286e9
              *                        //@>286e8
              sizeof(unsigned int));

  //@>177f6
  r                                    //@>177f5
    =                                  //@>17831
    5489U;

  //@>1cf9c
  state                                //@>17820
    [                                  //@>17832
    0]                                 //@>177f7
    =                                  //@>17833
    5489U;

  //@>1782a
  for (int                             //@>38350
       mti =                           //@>17812
       0;                              //@>38358
       mti <                           //@>17810
       623;                            //@>38355
       mti                             //@>3835c
       ++) {
    //@>177fc
    r                                  //@>177fb
      = ((                             //@>177fd
          r                            //@>2279a
          ^                            //@>177ff
          r                            //@>1781d
          >>                           //@>17834
          30U)                         //@>22797
         *                             //@>17835
         1812433253U                   //@>22932
         +                             //@>19851
         mti)                          //@>22933
      +                                //@>17803
      1U;

    //@>1cfac
    state                              //@>17821
      [                                //@>1985a
      mti                              //@>1996a
      +                                //@>17823
      1]                               //@>17804
      =                                //@>17808
      r;
  }

  //@>1cfa2
  state                                //@>17825
    [                                  //@>17836
    624]                               //@>1780a
    =                                  //@>17837
    624U;
}

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
