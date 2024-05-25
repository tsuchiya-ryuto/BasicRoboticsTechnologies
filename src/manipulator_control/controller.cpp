#include <cmath>
#include <iostream>

#include "manipulator_2d.h"
#include "pid_controller.h"

void display_arm(FILE *gp, std::vector<float> target, std::vector<std::vector<float>> joint_positions)
{
    fprintf(gp, "plot '-' with linespoints pt 7 title 'arm', '-' with points pt 7 title 'target'\n");
    fprintf(gp, "%f %f\n", joint_positions.at(0).at(0), joint_positions.at(0).at(1));
    fprintf(gp, "%f %f\n", joint_positions.at(1).at(0), joint_positions.at(1).at(1));
    fprintf(gp, "%f %f\n", joint_positions.at(2).at(0), joint_positions.at(2).at(1));
    fprintf(gp, "e\n");
    fprintf(gp, "%f %f\n", target.at(0), target.at(1));
    fprintf(gp, "e\n");
}

int main(int argc, char** argv)
{

    Manipulator::Arm2d arm;
    std::vector<float> joint_lengths = arm.get_joint_lengths();
    float arm_length = joint_lengths.at(0) + joint_lengths.at(1);
    FILE *gp;
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set xrange [%f:%f]\n", -1.1*arm_length, 1.1*arm_length);
    fprintf(gp, "set yrange [%f:%f]\n", -1.1*arm_length, 1.1*arm_length);
    fprintf(gp, "set nokey\n");
    fprintf(gp, "set size square\n");

    //std::vector<float> target_angles = {0, M_PI/2};
    //Manipulator::Arm2d arm2;
    //arm2.set_joint_angles(target_angles);
    //std::vector<float> target_position = arm2.get_joint_positions().at(2);
    std::vector<float> target_position = {0.5, 0.5};
    std::vector<float> Kp = {15, 15};
    std::vector<float> Ki = {0, 0};
    std::vector<float> Kd = {30, 30};
    float time_interval = 0.0005;
    float end_time = 10;
    Controller::PIDController pid(Kp, Ki, Kd, time_interval);

    std::vector<std::vector<float>> joint_positions;
    float time;
    while(time < end_time)
    {
        //std::vector<float> joint_angles = arm.get_joint_angles();
        //std::vector<float> torques = pid.control(target_angles, joint_angles);
        std::vector<float> end_effector_position = arm.get_joint_positions().at(2);
        std::vector<float> forces = pid.control(target_position, end_effector_position);
        //arm.torque_control(torques, time_interval);
        arm.force_control(forces, time_interval);
        joint_positions = arm.get_joint_positions();
        time += time_interval;
        display_arm(gp, target_position, joint_positions);
    }
    return 0;
}
