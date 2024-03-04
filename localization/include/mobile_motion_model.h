#ifndef ROBOT_MOTION_MODEL_H
#define ROBOT_MOTION_MODEL_H

#include <cmath>
#include <array>
#include <iostream>
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
    control_std = {0.3, 0.05, 0.1, 0.3};
  }

  Eigen::VectorXd state_transition()
  {
    float v, w;
    v = control.v;
    w = control.w;
    float epsilon = 10e-6;
    if (abs(w) < epsilon)
    {
      state.x += v * cos(state.theta) * time;
      state.y += v * sin(state.theta) * time;
      state.theta += w * time;
    }
    else
    {
      float ct = cos(state.theta);
      float st = sin(state.theta);
      float ctw = cos(state.theta + w*time);
      float stw = sin(state.theta + w*time);

      state.x += v/w*(stw - st);
      state.y += v/w*(-ctw + ct);
      state.theta += w*time;

      //std::cout << "x: " << state.x << " y: " << state.y << " theta: " << state.theta << std::endl;
    }

    return state.convert_to_eigen();
  }

  Eigen::MatrixXd jacobian_for_control()
  {
    Eigen::MatrixXd A(state.size(), control.size());
    float v, w;
    v = control.v;
    w = control.w;
    float epsilon = 10e-6;
    if(abs(w) < epsilon)
      w = epsilon;
    float ct = cos(state.theta);
    float st = sin(state.theta);
    float ctw = cos(state.theta + w*time);
    float stw = sin(state.theta + w*time);
    A << \
      (stw - st)/w, -v/(w*w)*(stw - st) + v/w*time*ctw,
      (-ctw + ct)/w, -v/(w*w)*(-ctw + ct) + v/w*time*stw,
      0, time;
    return A;
  }

  Eigen::MatrixXd jacobian_for_state()
  {
    Eigen::MatrixXd F(state.size(), state.size());
    float v, w;
    v = control.v;
    w = control.w;
    float epsilon = 10e-6;
    if(abs(w) < epsilon)
      w = epsilon;
    float ct = cos(state.theta);
    float st = sin(state.theta);
    float ctw = cos(state.theta + w*time);
    float stw = sin(state.theta + w*time);

    F <<  \
      1, 0, v/w*(ctw - ct),
      0, 1, v/w*(stw - st),
      0, 0, 1;
    return F;
  }

  Eigen::MatrixXd control_covariance()
  {
    Eigen::MatrixXd M(control.size(), control.size());
    float v, w;
    v = control.v;
    w = control.w;
    float vv, vw, wv, ww;
    vv = control_std.at(0)*control_std.at(0);
    vw = control_std.at(1)*control_std.at(1);
    wv = control_std.at(2)*control_std.at(2);
    ww = control_std.at(3)*control_std.at(3);
    /*
    M << \
      vv*abs(v)/time + vw*abs(w)/time, 0,
      0, wv*abs(v)/time + ww*abs(w)/time;
    */
    M << \
      0.3, 0,
      0, 0.3;
    return M;
  }

private:
  State state;
  Control control;
  float time;
  std::array<float, 4> control_std;
};

#endif