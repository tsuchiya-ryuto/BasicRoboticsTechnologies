#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mobile_motion_model.h>
#include <gps_observation_model.h>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

namespace BayesianFilter
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter(Vector initial_state, Vector initial_covariance)
  :motion_model(), observation_model()
  {
    mean.resize(initial_state.size());
    mean = initial_state;
    covariance.resize(initial_state.size(), initial_state.size());
    for(size_t i = 0; i < initial_covariance.size(); i++)
      covariance(i, i) = initial_covariance(i);
  }

  ExtendedKalmanFilter(Vector initial_state, Matrix initial_covariance)
  :motion_model(), observation_model()
  {
    mean.resize(initial_state.size());
    mean = initial_state;
    covariance.resize(initial_state.size(), initial_state.size());
    covariance = initial_covariance;
  }

  void control_update(Vector control, float time)
  {
    motion_model.set_states(mean, control, time);
    Matrix F, A, M;
    F = motion_model.jacobian_for_state();
    A = motion_model.jacobian_for_control();
    M = motion_model.control_covariance();

    mean = motion_model.state_transition();
    covariance = F*covariance*F.transpose() + A*M*A.transpose();
  }

  void observation_update(Vector observation, Vector observation_covariance)
  {
    Matrix H, Q, S, K;
    observation_model.set_observations(observation, observation_covariance);
    H = observation_model.observation_equation();
    Q = observation_model.observation_covariance();
    S = H*covariance*H.transpose() + Q;
    K = covariance * H.transpose() * S.inverse(); // Kalman Gain

    mean += K * (mean - H*observation);
    Matrix I = Matrix::Identity(mean.size(), mean.size());
    covariance = (I - K*H)*covariance;
  }

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