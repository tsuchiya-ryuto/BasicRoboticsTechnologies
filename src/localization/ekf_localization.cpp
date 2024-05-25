#include "extended_kalman_filter.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <random>
#include <vector>
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv)
{
  Eigen::VectorXd state(3);
  state << 0.0, 0.0, 0.0;
  Eigen::MatrixXd covariance(3,3);
  covariance << \
    0.05, 0.0, 0.0,
    0.0, 0.05, 0.0,
    0.0, 0.0, 0.05*M_PI;
  Filter::ExtendedKalmanFilter ekf(state, covariance);

  FILE *gp;
  gp = popen("gnuplot -persist", "w");
  //fprintf(gp, "set term gif animate optimize delay 10 size 500, 500\n");
  //fprintf(gp, "set output 'ekf_localization.gif'\n");
  fprintf(gp, "set key outside\n");
  //fprintf(gp, "set xrange [-1.0:1.0]\n");
  //fprintf(gp, "set yrange [-1.0:1.0]\n");
  fprintf(gp, "set size square\n");

  // uniform motion
  float v, w;
  v = 0.5;
  w = 0.3;
  float time_interval = 0.01;
  Eigen::VectorXd ideal_control(2);
  ideal_control << v, w;
  Eigen::VectorXd ideal_state(3), odometry(3);
  ideal_state << 0.0, 0.0, 0.0;
  odometry << 0.0, 0.0, 0.0;

  MotionModel motion_model;
  std::random_device gen;
  std::default_random_engine engine(gen());
  std::normal_distribution<float> vel_dist(0.0, 0.1);
  std::normal_distribution<float> ang_dist(0.0, 0.1*M_PI);

  Eigen::VectorXd obs_cov(3);
  obs_cov << 0.05, 0.05, 0.2*M_PI;
  std::normal_distribution<float> gps_x(0.0, obs_cov(0));
  std::normal_distribution<float> gps_y(0.0, obs_cov(1));
  std::normal_distribution<float> gps_theta(0.0, obs_cov(2));

  std::vector<Eigen::VectorXd> actual_trajectory;
  std::vector<Eigen::VectorXd> trajectory;
  std::vector<Eigen::VectorXd> wheel_odometry;
  std::vector<Eigen::VectorXd> gps;
  trajectory.push_back(state);
  wheel_odometry.push_back(state);
  actual_trajectory.push_back(state);
  
  for(int i = 0; i < 1000; i++)
  {
    // ideal (no noise)
    motion_model.set_states(ideal_state, ideal_control, time_interval);
    ideal_state = motion_model.state_transition();
    actual_trajectory.push_back(ideal_state);

    // ekf
    Eigen::VectorXd control(2);
    control << v + vel_dist(engine), w + ang_dist(engine);
    ekf.control_update(control, time_interval);

    Eigen::VectorXd obs(3);
    obs << \
      ideal_state(0) + gps_x(engine), 
      ideal_state(1) + gps_y(engine), 
      ideal_state(2) + gps_theta(engine);
    ekf.observation_update(obs, obs_cov);
    gps.push_back(obs);

    auto mean = ekf.get_mean();
    auto covariance = ekf.get_covariance();
    trajectory.push_back(mean);

    // wheel odometry
    motion_model.set_states(odometry, control, time_interval);
    odometry = motion_model.state_transition();
    wheel_odometry.push_back(odometry);

    // plot
    fprintf(gp, "set parametric\n");
    fprintf(gp, "plot [0:2*pi] %f + %f*cos(t), %f + %f*sin(t) title 'error', ", mean(0), covariance(0, 0), mean(1), covariance(1, 1));
    fprintf(gp, "'-' with lines linecolor 'red' title 'actual', ");
    fprintf(gp, " '-' with lines linecolor 'blue' title 'estimated', ");
    fprintf(gp, " '-' with lines linecolor 'green' title 'wheel odometry', ");
    fprintf(gp, " '-' with points pt 7 ps 0.3 linecolor 'black' title 'gps'\n");
    for(auto t : actual_trajectory) fprintf(gp, "%f %f\n", t(0), t(1)); fprintf(gp, "e\n");
    for(auto t : trajectory) fprintf(gp, "%f %f\n", t(0), t(1)); fprintf(gp, "e\n");
    for(auto t : wheel_odometry) fprintf(gp, "%f %f\n", t(0), t(1)); fprintf(gp, "e\n");
    for(auto t : gps) fprintf(gp, "%f %f\n", t(0), t(1)); fprintf(gp, "e\n");
  }
  pclose(gp);

  return 0;
}