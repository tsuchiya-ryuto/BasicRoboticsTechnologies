#ifndef MANIPULATOR_2d_H
#define MANIPULATOR_2d_H

#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

namespace Manipulator
{

class Arm2d {

public:
    Arm2d();
    ~Arm2d();

    vector<vector<float>> forward_kinematics();
    void inverse_kinematics(const vector<float>& end_effector_position);
    void torque_control(const vector<float>& torques, const float time);
    void force_control(const vector<float>& forces, const float time);

    Eigen::Matrix2d jacobian();
    Eigen::Matrix2d mass_matrix();
    Eigen::Vector2d coriolis_matrix();
    Eigen::Vector2d gravity_matrix();

    void set_joint_angles(const vector<float>& joint_angles); 
    void set_joint_lengths(const vector<float>& joint_lengths);
    void set_joint_masses(const vector<float>& joint_masses);

    vector<float> get_joint_angles();
    vector<float> get_joint_lengths();
    vector<vector<float>> get_joint_positions();


private:
    float theta1, theta2;
    float theta1_dot, theta2_dot;
    float l1, l2;
    float m1, m2;
    float lg1, lg2;
    vector<float> base_position;
    
    float g = 9.81;

};

}

#endif