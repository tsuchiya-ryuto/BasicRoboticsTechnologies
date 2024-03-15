#ifndef ROBOT_MOTION_MODEL_H
#define ROBOT_MOTION_MODEL_H

#include <cmath>
#include <array>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <model/mobile_robot_states.h>

class MotionModel
{
public:
  MotionModel();

  void set_states(const Eigen::VectorXd& xx, const Eigen::VectorXd& uu, const float tt);
  Eigen::VectorXd state_transition();
  Eigen::MatrixXd jacobian_for_control();
  Eigen::MatrixXd jacobian_for_state();
  Eigen::MatrixXd control_covariance();

private:
  State state;
  Control control;
  float time;
  std::array<float, 4> control_std;
};

#endif