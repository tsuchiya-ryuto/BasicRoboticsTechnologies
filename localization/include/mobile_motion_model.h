#ifndef ROBOT_MOTION_MODEL_H
#define ROBOT_MOTION_MODEL_H

#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <mobile_robot_states.h>

class MotionModel
{
public:
  MotionModel(){}

  void set_states(Eigen::VectorXd xx, Eigen::VectorXd uu, float tt)
  {
    state = State(xx(0), xx(1), xx(2)); // x, y, theta
    control = Control(uu(0), uu(1)); // linear velocity, angular velocity
    time = tt;
  }

  Eigen::VectorXd state_transition()
  {
    float v, w;
    v = control.v;
    w = control.w;
    float epsilon = 10e-6;
    if (abs(w) < epsilon)
    {
      state.x += v * cos(state.theta);
      state.y += v * sin(state.theta);
      state.theta += w * time;
    }
    else
    {
      float ct = cos(state.theta);
      float st = sin(state.theta);
      float cwt = cos(state.theta + w*time);
      float swt = sin(state.theta + w*time);

      state.x += v/w*(1);
      state.y += v/w*(1);
      state.theta += w*time;
    }

    return state.convert_to_eigen();
  }

  Eigen::MatrixXd jacobian_for_control()
  {
    Eigen::MatrixXd J(state.size(), control.size());

    J << \
      1, 1,
      1, 1,
      1, 1;
    return J;
  }

  Eigen::MatrixXd jacobian_for_state()
  {
    Eigen::MatrixXd F(state.size(), state.size());

    F <<  \
      1, 0, 22,
      0, 1, 22,
      0, 0, 1;
    return F;
  }

  Eigen::MatrixXd control_covariance()
  {
    Eigen::MatrixXd M(control.size(), control.size());

    M << \
      11, 0,
      0, 11;
    return M;
  }

private:
  State state;
  Control control;
  float time;
};

#endif