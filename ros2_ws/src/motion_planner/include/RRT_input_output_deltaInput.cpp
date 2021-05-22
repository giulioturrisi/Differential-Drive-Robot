//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RRT_input_output_deltaInput.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 22-May-2021 11:59:19
//

// Include Files
#include "RRT_input_output_deltaInput.h"
#include "rand.h"
#include "coder_array.h"
#include <cmath>

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions
//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// obj.nodes = vertcat(obj.nodes,new_node);
// coder.varsize('obj.nodes');
// Arguments    : const double new_node[6]
// Return Type  : void
//
void RRT_input_output_deltaInput::add_nodes(const double new_node[6])
{
  int obj;
  this->numberIter++;
  obj = static_cast<int>(this->numberIter);
  for (int i = 0; i < 6; i++) {
    this->nodes[(obj + this->nodes.size(0) * i) - 1] = new_node[i];
  }
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : const double node_to_check[6]
// Return Type  : double
//
double RRT_input_output_deltaInput::check_collision(const double node_to_check[6])
  const
{
  double d;
  double good;
  double scale;
  short i;
  scale = 1.0 / this->resolution;
  d = rt_roundd_snf(node_to_check[0] * scale);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      i = static_cast<short>(d);
    } else {
      i = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    i = MAX_int16_T;
  } else {
    i = 0;
  }

  if (i < 1) {
    good = 0.0;
  } else {
    double d1;
    d1 = rt_roundd_snf(node_to_check[1] * scale);
    if (d1 < 32768.0) {
      if (d1 >= -32768.0) {
        i = static_cast<short>(d1);
      } else {
        i = MIN_int16_T;
      }
    } else if (d1 >= 32768.0) {
      i = MAX_int16_T;
    } else {
      i = 0;
    }

    if (i < 1) {
      good = 0.0;
    } else if ((std::abs(node_to_check[0]) > this->map_limit[0] * scale - 5.0) ||
               (std::abs(node_to_check[1]) > this->map_limit[1] * scale - 5.0))
    {
      good = 0.0;

      // elseif(obj.map(top(1),top(2)) < 250 | obj.map(bottom(1),bottom(2)) < 250 | obj.map(left(1),left(2)) < 250 | obj.map(right(1),right(2)) < 250) 
      //     good = 0;
    } else {
      short i1;
      if (d < 32768.0) {
        if (d >= -32768.0) {
          i = static_cast<short>(d);
        } else {
          i = MIN_int16_T;
        }
      } else if (d >= 32768.0) {
        i = MAX_int16_T;
      } else {
        i = 0;
      }

      if (d1 < 32768.0) {
        if (d1 >= -32768.0) {
          i1 = static_cast<short>(d1);
        } else {
          i1 = MIN_int16_T;
        }
      } else if (d1 >= 32768.0) {
        i1 = MAX_int16_T;
      } else {
        i1 = 0;
      }

      good = (this->map[(i + this->map.size(0) * (i1 - 1)) - 1] >= 250);
    }
  }

  // end methods
  // end class
  return good;
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : const double new_node[6]
// Return Type  : double
//
double RRT_input_output_deltaInput::check_goal(const double new_node[6]) const
{
  double a;
  double finish;
  a = new_node[0] - this->goal[0];
  if (a * a < 0.2) {
    a = new_node[1] - this->goal[1];
    if (a * a < 0.2) {
      finish = 1.0;
    } else {
      finish = 0.0;
    }
  } else {
    finish = 0.0;
  }

  return finish;
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : double near_index
//                const double desired_node[3]
//                double new_node[6]
// Return Type  : void
//
void RRT_input_output_deltaInput::choose_primitives(double near_index, const
  double desired_node[3], double new_node[6]) const
{
  double near_node[6];
  for (int i = 0; i < 6; i++) {
    near_node[i] = this->nodes[(static_cast<int>(near_index) + this->nodes.size
      (0) * i) - 1];
  }

  double u1;
  double u2;

  // best_control = [obj.k obj.k];
  // best_control = [0 0];
  // to add dynamic
  u1 = (this->k[0] * (desired_node[0] - near_node[0]) + this->k[1] * -near_node
        [4]) + near_node[4];
  u2 = (this->k[1] * (desired_node[1] - near_node[1]) + this->k[1] * -near_node
        [5]) + near_node[5];

  //  + w*obj.dt;
  new_node[0] = near_node[0] + u1 * this->dt;
  new_node[1] = near_node[1] + u2 * this->dt;
  new_node[2] = near_node[2] + (u2 * std::cos(near_node[2]) - u1 * std::sin
    (near_node[2])) * this->dt / 0.05;
  new_node[3] = near_index;
  new_node[4] = u1;
  new_node[5] = u2;
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : const double new_node[3]
// Return Type  : double
//
double RRT_input_output_deltaInput::find_nearest(const double new_node[3]) const
{
  double node[6];
  double best_distance;
  double near_index;
  int i;

  // number_of_nodes = size(obj.nodes);
  // number_of_nodes = number_of_nodes(1);
  near_index = 1.0;
  best_distance = 10000.0;
  i = static_cast<int>(this->numberIter);
  for (int b_k = 0; b_k < i; b_k++) {
    double distance;
    for (int i1 = 0; i1 < 6; i1++) {
      node[i1] = this->nodes[b_k + this->nodes.size(0) * i1];
    }

    double a;
    double b_a;
    distance = node[0] - new_node[0];
    a = node[1] - new_node[1];
    b_a = node[2] - new_node[2];
    distance = std::sqrt((distance * distance + a * a) + 0.0 * (b_a * b_a));
    if (distance < best_distance) {
      best_distance = distance;
      near_index = static_cast<double>(b_k) + 1.0;
    }
  }

  return near_index;
}

//
// Arguments    : const double initial_state[6]
//                double sampling_time
//                const double limit[2]
//                const double b_goal[2]
//                const coder::array<unsigned char, 2U> &b_map
//                double b_resolution
//                double b_maxIter
// Return Type  : RRT_input_output_deltaInput *
//
RRT_input_output_deltaInput *RRT_input_output_deltaInput::init(const double
  initial_state[6], double sampling_time, const double limit[2], const double
  b_goal[2], const coder::array<unsigned char, 2U> &b_map, double b_resolution,
  double b_maxIter)
{
  RRT_input_output_deltaInput *obj;
  int i;
  int loop_ub;
  obj = this;

  // RRT_PRIMITIVES Summary of this class goes here
  //    Detailed explanation goes here
  // RRT_PRIMITIVES Construct an instance of this class
  //    Detailed explanation goes here
  // obj.nodes = [initial_state(1) initial_state(2) initial_state(3) 0 0 0];
  obj->dt = sampling_time;
  obj->map_limit[0] = limit[0];
  obj->map_limit[1] = limit[1];
  obj->goal[0] = b_goal[0];
  obj->goal[1] = b_goal[1];
  obj->map.set_size(b_map.size(0), b_map.size(1));
  loop_ub = b_map.size(0) * b_map.size(1);
  for (i = 0; i < loop_ub; i++) {
    obj->map[i] = b_map[i];
  }

  obj->k[0] = 0.477;
  obj->k[1] = 0.5449;
  obj->resolution = b_resolution;
  obj->maxIter = b_maxIter;
  obj->numberIter = 1.0;
  obj->nodes.set_size((static_cast<int>(b_maxIter)), 6);
  loop_ub = static_cast<int>(b_maxIter) * 6;
  for (i = 0; i < loop_ub; i++) {
    obj->nodes[i] = 0.0;
  }

  obj->nodes[0] = initial_state[0];
  obj->nodes[obj->nodes.size(0)] = initial_state[1];
  obj->nodes[obj->nodes.size(0) * 2] = initial_state[2];
  obj->nodes[obj->nodes.size(0) * 3] = 0.0;
  obj->nodes[obj->nodes.size(0) * 4] = 0.0;
  obj->nodes[obj->nodes.size(0) * 5] = 0.0;
  return obj;
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : double desired_node[3]
// Return Type  : void
//
void RRT_input_output_deltaInput::sample(double desired_node[3]) const
{
  double sample_goal_prob;
  sample_goal_prob = coder::b_rand();
  if (sample_goal_prob > 0.8) {
    desired_node[0] = this->goal[0];
    desired_node[1] = this->goal[1];
    desired_node[2] = 0.0;
  } else {
    double rand_y;
    double rand_z;
    sample_goal_prob = coder::b_rand() * this->map_limit[0];
    rand_y = coder::b_rand() * this->map_limit[1];
    rand_z = (coder::b_rand() - 0.5) * 3.1415926535897931;
    desired_node[0] = sample_goal_prob;
    desired_node[1] = rand_y;
    desired_node[2] = rand_z;
  }
}

//
// METHOD1 Summary of this method goes here
//    Detailed explanation goes here
// Arguments    : double b_index
//                coder::array<double, 2U> &path
//                double *size_path
// Return Type  : void
//
void RRT_input_output_deltaInput::take_path(double b_index, coder::array<double,
  2U> &path, double *size_path) const
{
  double parent[6];
  int i;
  int loop_ub;
  for (i = 0; i < 6; i++) {
    parent[i] = this->nodes[(static_cast<int>(b_index) + this->nodes.size(0) * i)
      - 1];
  }

  path.set_size((static_cast<int>(this->maxIter)), 6);
  loop_ub = static_cast<int>(this->maxIter) * 6;
  for (i = 0; i < loop_ub; i++) {
    path[i] = 1.0;
  }

  for (i = 0; i < 6; i++) {
    path[path.size(0) * i] = parent[i];
  }

  *size_path = 0.0;

  // path = [final_node];
  loop_ub = 1;
  while ((loop_ub - 1 <= static_cast<int>(b_index) - 1) && (!(parent[3] == 0.0)))
  {
    double b_parent;
    b_parent = parent[3];

    // path = vertcat(path,parent);
    for (i = 0; i < 6; i++) {
      double d;
      d = this->nodes[(static_cast<int>(b_parent) + this->nodes.size(0) * i) - 1];
      parent[i] = d;
      path[loop_ub + path.size(0) * i] = d;
    }

    (*size_path)++;
    loop_ub++;
  }
}

//
// File trailer for RRT_input_output_deltaInput.cpp
//
// [EOF]
//
