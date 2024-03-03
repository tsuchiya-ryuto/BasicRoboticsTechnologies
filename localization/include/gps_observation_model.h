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
  
  void set_observations(Eigen::VectorXd observation, Eigen::VectorXd variances){}
  Eigen::MatrixXd observation_equation()
  {
    Eigen::MatrixXd H;
    H << \
      1, 0, 0,
      0, 1, 0,
      0, 0, 1;
    return H;
  }

  Eigen::MatrixXd observation_covariance()
  {
    Eigen::MatrixXd Q;
    Q << \
      0.1, 0, 0,
      0, 0.1, 0,
      0, 0, 0.1;
    return Q;
  }

private:

};


#endif