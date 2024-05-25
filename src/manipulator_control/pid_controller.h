#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>
#include <cmath>
#include <iostream>

namespace Controller
{

class PIDController
{
public:
    PIDController(
        const std::vector<float> Kp, 
        const std::vector<float> Ki,
        const std::vector<float> Kd,
        const float dt);
    ~PIDController();
    std::vector<float> control(const std::vector<float>& x_d, const std::vector<float>& x);

private:
    std::vector<float> Kp, Ki, Kd;
    float dt;
    std::vector<float> error_sum, prev_error;

};

}

#endif
