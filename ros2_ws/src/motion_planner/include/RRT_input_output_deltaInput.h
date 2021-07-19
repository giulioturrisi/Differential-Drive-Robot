//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RRT_input_output_deltaInput.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 19-Jul-2021 23:22:17
//
#ifndef RRT_INPUT_OUTPUT_DELTAINPUT_H
#define RRT_INPUT_OUTPUT_DELTAINPUT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class RRT_input_output_deltaInput
{
 public:
  RRT_input_output_deltaInput *init(const double initial_state[6], double
    sampling_time, const double limit[2], const double b_goal[2], const coder::
    array<double, 2U> &b_map, double b_resolution, double b_maxIter);
  void sample(double desired_node[3]) const;
  double find_nearest(const double new_node[3]) const;
  void choose_primitives(double near_index, const double desired_node[3], double
    new_node[6]) const;
  double check_collision(const double node_to_check[6]) const;
  void add_nodes(const double new_node[6]);
  double check_goal(const double new_node[6]) const;
  void take_path(double b_index, coder::array<double, 2U> &path, double
                 *size_path) const;
  double b_find_nearest(const double new_node[6]) const;
  coder::array<double, 2U> nodes;
  double dt;
  double map_limit[2];
  double goal[2];
  coder::array<double, 2U> map;
  double k[2];
  double resolution;
  double maxIter;
  double numberIter;
};

#endif

//
// File trailer for RRT_input_output_deltaInput.h
//
// [EOF]
//
