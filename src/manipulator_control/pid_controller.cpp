#include "pid_controller.h"

namespace Controller
{
    PIDController::PIDController(
        const std::vector<float> Kp,
        const std::vector<float> Ki,
        const std::vector<float> Kd,
        const float dt)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->dt = dt;
        int state_size = Kp.size();
        this->error_sum.resize(state_size);
        this->prev_error.resize(state_size);
    }

    PIDController::~PIDController(){}

    std::vector<float> PIDController::control(
        const std::vector<float>& x_d, 
        const std::vector<float>& x
    )
    {
        std::vector<float> error(x.size()), error_diff(x.size());
        for(int i = 0; i < x.size(); i++)
        {
            error.at(i) = x_d.at(i) - x.at(i);
            error_sum.at(i) += (prev_error.at(i) + error.at(i)) / 2 * dt;
            error_diff.at(i) = (error.at(i) - this->prev_error.at(i))/this->dt;

            prev_error.at(i) = error.at(i);
        }

        std::vector<float> u(x.size());
        for(int i = 0; i < x.size(); i++)
            u.at(i) = \
                Kp.at(i)*error.at(i) 
                + Ki.at(i)*error_sum.at(i) 
                + Kd.at(i)*error_diff.at(i);
        return u;
    }

}
