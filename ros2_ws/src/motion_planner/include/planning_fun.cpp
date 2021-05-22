//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: planning_fun.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 22-May-2021 11:59:19
//

// Include Files
#include "planning_fun.h"
#include "RRT_input_output_deltaInput.h"
#include "planning_fun_data.h"
#include "planning_fun_initialize.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const double state_robot[6]
//                double dt
//                const double limit[2]
//                const double goal[2]
//                const coder::array<unsigned char, 2U> &image
//                double resolution
//                double maxIter
//                coder::array<double, 2U> &final_path
// Return Type  : void
//
void planning_fun(const double state_robot[6], double dt, const double limit[2],
                  const double goal[2], const coder::array<unsigned char, 2U>
                  &image, double resolution, double maxIter, coder::array<double,
                  2U> &final_path)
{
  RRT_input_output_deltaInput RRT;
  coder::array<double, 2U> path;
  double new_node[6];
  double desired_node[3];
  double size_path;
  int i;
  int j;
  bool exitg1;
  if (!isInitialized_planning_fun) {
    planning_fun_initialize();
  }

  RRT.init(state_robot, dt, limit, goal, image, resolution, maxIter);
  path.set_size(1, 6);
  for (i = 0; i < 6; i++) {
    path[i] = state_robot[i];
  }

  size_path = 1.0;

  // RRT loop
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= static_cast<int>(maxIter) - 1)) {
    double near_index;
    RRT.sample(desired_node);
    near_index = RRT.find_nearest(desired_node);
    RRT.choose_primitives(near_index, desired_node, new_node);

    // check collision
    near_index = RRT.check_collision(new_node);
    if (near_index == 1.0) {
      RRT.add_nodes(new_node);
    }

    near_index = RRT.check_goal(new_node);
    if (near_index == 1.0) {
      RRT.take_path(new_node[3], path, (&size_path));
      exitg1 = true;
    } else {
      j++;
    }
  }

  j = static_cast<int>(size_path + 1.0);
  final_path.set_size((static_cast<int>(size_path + 1.0)), 6);
  for (i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < j; i1++) {
      final_path[i1 + final_path.size(0) * i] = path[i1 + path.size(0) * i];
    }
  }
}

//
// File trailer for planning_fun.cpp
//
// [EOF]
//
