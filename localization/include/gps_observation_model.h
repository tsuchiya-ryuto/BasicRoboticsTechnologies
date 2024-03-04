#ifndef GPS_OBSERVATION_MODEL_H
#define GPS_OBSERVATION_MODEL_H

#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <mobile_robot_states.h>

class ObservationModel
{
public:
  ObservationModel(){}
  
  void set_observations(Eigen::VectorXd observation, Eigen::VectorXd variances)
  {
    obs = observation;
    cov = variances;
  }

  Eigen::MatrixXd observation_equation()
  {
    Eigen::MatrixXd H(3,3);
    H << \
      1, 0, 0,
      0, 1, 0,
      0, 0, 1;
    return H;
  }

  Eigen::MatrixXd observation_covariance()
  {
    Eigen::MatrixXd Q(3,3);
    Q << \
      cov(0), 0, 0,
      0, cov(1), 0,
      0, 0, cov(2);
    return Q;
  }

private:
  Eigen::VectorXd obs;
  Eigen::VectorXd cov;

};


#endif