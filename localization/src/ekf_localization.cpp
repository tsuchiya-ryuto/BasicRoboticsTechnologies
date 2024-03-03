#include <extended_kalman_filter.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main(int argc, char** argv)
{
  Eigen::VectorXd state(3);
  state << 0.0, 0.0, 0.0;
  Eigen::MatrixXd covariance(3,3);
  covariance << \
    0.1, 0.0, 0.0,
    0.0, 0.1, 0.0,
    0.0, 0.0, 0.1;
  BayesianFilter::ExtendedKalmanFilter(state, covariance);
}