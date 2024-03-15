#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <model/mobile_motion_model.h>
#include <model/gps_observation_model.h>
#include <iostream>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

namespace Filter
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter(const Vector& initial_state, const Vector& initial_covariance);
  ExtendedKalmanFilter(const Vector& initial_state, const Matrix& initial_covariance);

  void control_update(const Vector& control, const float time);
  void observation_update(const Vector& observation, const Vector& observation_covariance);

  Vector get_mean();
  Matrix get_covariance();

private:
  // parameters of gaussian
  Vector mean;
  Matrix covariance;
  time_t last_time;
  MotionModel motion_model;
  ObservationModel observation_model;
};

}

#endif