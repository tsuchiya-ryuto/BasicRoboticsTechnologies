#include "extended_kalman_filter.h"

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

namespace Filter
{

ExtendedKalmanFilter::ExtendedKalmanFilter(const Vector& initial_state, const Vector& initial_covariance)
:motion_model(), observation_model()
{
  mean.resize(initial_state.size());
  mean = initial_state;
  covariance.resize(initial_state.size(), initial_state.size());
  for(size_t i = 0; i < initial_covariance.size(); i++)
    covariance(i, i) = initial_covariance(i);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const Vector& initial_state, const Matrix& initial_covariance)
:motion_model(), observation_model()
{
  mean.resize(initial_state.size());
  mean = initial_state;
  covariance.resize(initial_state.size(), initial_state.size());
  covariance = initial_covariance;
}

void ExtendedKalmanFilter::control_update(const Vector& control, const float time)
{
  motion_model.set_states(mean, control, time);
  Matrix F, A, M;
  F = motion_model.jacobian_for_state();
  A = motion_model.jacobian_for_control();
  M = motion_model.control_covariance();

  mean = motion_model.state_transition();
  covariance = F*covariance*F.transpose() + A*M*A.transpose();
}

void ExtendedKalmanFilter::observation_update(const Vector& observation, const Vector& observation_covariance)
{
  Matrix H, Q, S, K;
  observation_model.set_observations(observation, observation_covariance);
  H = observation_model.observation_equation();
  Q = observation_model.observation_covariance();

  // S is just for making easier to calculate Kalman Gain
  S = H*covariance*H.transpose() + Q;
  K = covariance * H.transpose() * S.inverse(); // Kalman Gain

  mean += K * (H*observation - mean);
  Matrix I = Matrix::Identity(mean.size(), mean.size());
  covariance = (I - K*H)*covariance;
}

Vector ExtendedKalmanFilter::get_mean() {return mean;}
Matrix ExtendedKalmanFilter::get_covariance() {return covariance;}

}