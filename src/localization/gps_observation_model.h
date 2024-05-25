#ifndef GPS_OBSERVATION_MODEL_H
#define GPS_OBSERVATION_MODEL_H

#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "mobile_robot_states.h"

class ObservationModel
{
public:
  ObservationModel(){}
  
  void set_observations(const Eigen::VectorXd& observation, const Eigen::VectorXd& variances);

  Eigen::MatrixXd observation_equation();
  Eigen::MatrixXd observation_covariance();

private:
  Eigen::VectorXd obs;
  Eigen::VectorXd cov;

};


#endif