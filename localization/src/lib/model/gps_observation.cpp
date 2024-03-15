#include <model/gps_observation_model.h>

void ObservationModel::set_observations(const Eigen::VectorXd& observation, const Eigen::VectorXd& variances)
{
  obs = observation;
  cov = variances;
}

Eigen::MatrixXd ObservationModel::observation_equation()
{
  Eigen::MatrixXd H(3,3);
  H << \
    1, 0, 0,
    0, 1, 0,
    0, 0, 1;
  return H;
}

Eigen::MatrixXd ObservationModel::observation_covariance()
{
  Eigen::MatrixXd Q(3,3);
  Q << \
    cov(0), 0, 0,
    0, cov(1), 0,
    0, 0, cov(2);
  return Q;
}
