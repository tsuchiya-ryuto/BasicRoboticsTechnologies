#include <manipulator/manipulator_2d.h>

using namespace std;

namespace Manipulator
{

Arm2d::Arm2d()
{
    theta1 = 0;
    theta2 = 0;
    theta1_dot = 0;
    theta2_dot = 0;
    l1 = 0.5;
    l2 = 0.5;
    m1 = 1.0;
    m2 = 1.0;
    lg1 = l1/2;
    lg2 = l2/2;
    this->base_position = {0, 0};
}

Arm2d::~Arm2d(){}

vector<vector<float>> Arm2d::forward_kinematics()
{
    float x1 = l1 * cos(theta1);
    float y1 = l1 * sin(theta1);
    float x2 = x1 + l2 * cos(theta1 + theta2);
    float y2 = y1 + l2 * sin(theta1 + theta2);
    vector<float> joint_position1 = {x1, y1};
    vector<float> joint_position2 = {x2, y2};
    return {this->base_position, joint_position1, joint_position2};
}

void Arm2d::inverse_kinematics(const vector<float>& end_effector_position)
{
    Eigen::Matrix2d J = jacobian();

}

void Arm2d::torque_control(const vector<float>& torques, const float time)
{
    // euler method
    Eigen::Vector2d q = {theta1, theta2};
    Eigen::Vector2d q_dot = {theta1_dot, theta2_dot};
    Eigen::Vector2d tau = {torques.at(0), torques.at(1)};

    Eigen::Matrix2d M = mass_matrix();
    Eigen::Vector2d h = coriolis_matrix();
    Eigen::Vector2d g_th = gravity_matrix();
    tau += g_th;
    Eigen::Vector2d q_2dot = M * (tau - h - g_th);

    q = q + q_dot * time;
    q_dot = q_dot + q_2dot * time;

    theta1 = q(0);
    theta2 = q(1);
    theta1_dot = q_dot(0);
    theta2_dot = q_dot(1);
}

void Arm2d::force_control(const vector<float>& forces, const float time)
{
    Eigen::Vector2d force = {forces.at(0), forces.at(1)};
    Eigen::Matrix2d J = jacobian();
    Eigen::Vector2d torque = J * force;

    std::vector<float> torque_vec = {static_cast<float>(torque(0)), static_cast<float>(torque(1))};
    torque_control(torque_vec, time);
}

Eigen::Matrix2d Arm2d::jacobian()
{
    float theta12 = theta1 + theta2;
    Eigen::Matrix2d J;
    J << -l1*sin(theta1) - l2*sin(theta12), -l2*sin(theta12),
            l1*cos(theta1) + l2*cos(theta12), l2*cos(theta12);
    return J;
}

Eigen::Matrix2d Arm2d::mass_matrix()
{
    float q1 = theta1;
    float q2 = theta2;
    Eigen::Matrix2d M;
    M(0,0) = m1*lg1*lg1 + l1 + m2*(l1*l1 + lg2*lg2 + 2*l1*lg2*cos(theta2)) + l2;
    M(0,1) = m2*(lg2*lg2 + l1*lg2*cos(theta2)) + l2;
    M(1,0) = M(0,1);
    M(1,1) = m2*lg2*lg2 + l2;

    return M;
}

Eigen::Vector2d Arm2d::coriolis_matrix()
{
    float h122 = -m2*l1*lg2*sin(theta2);
    float h112 = h122;
    float h211 = -h122;

    Eigen::Vector2d h;
    h(0) = h122*theta2_dot*theta2_dot + h112*theta1_dot*theta2_dot;
    h(1) = h211*theta1_dot*theta1_dot;

    return h;
}

Eigen::Vector2d Arm2d::gravity_matrix()
{
    Eigen::Vector2d g_th;
    g_th(0) = m1*g*lg1*cos(theta1) + m2*g*(l1*cos(theta1) + lg2*cos(theta1 + theta2));
    g_th(1) = m2*g*lg2*cos(theta1 + theta2);
    return g_th;
}

void Arm2d::set_joint_angles(const vector<float>& joint_angles) 
{
    theta1 = joint_angles.at(0);
    theta2 = joint_angles.at(1);
}
void Arm2d::set_joint_lengths(const vector<float>& joint_lengths)
{
    l1 = joint_lengths.at(0);
    l2 = joint_lengths.at(1);
}
void Arm2d::set_joint_masses(const vector<float>& joint_masses)
{
    m1 = joint_masses.at(0);
    m2 = joint_masses.at(1);
} 

vector<float> Arm2d::get_joint_angles() {return {theta1, theta2};}
vector<float> Arm2d::get_joint_lengths() {return {l1, l2};}

vector<vector<float>> Arm2d::get_joint_positions() 
{
    vector<vector<float>> joint_positions;
    joint_positions = forward_kinematics();
    return joint_positions;
}

}
