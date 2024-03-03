#ifndef MOBILE_ROBOT_STATES_H
#define MOBILE_ROBOT_STATES_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

struct State
{
  float x, y, theta;
  State(): x(0.0), y(0.0), theta(0.0) {}
  State(float xx, float yy, float tt): x(xx), y(yy), theta(tt) {}

  bool operator== (const State& s)
  {
    float epsilon = 10e-6;
    if(s.x > x - epsilon && s.x < x + epsilon)
    {
      if(s.y > y - epsilon && s.y < y + epsilon)
      {
        if(s.theta > theta - epsilon && s.theta < theta + epsilon)
        {
          return true;
        }
      }
    }
    
    return false;
  }

  Eigen::VectorXd convert_to_eigen()
  {
    Eigen::VectorXd ans(this->size());
    ans << x, y, theta;
    return ans;
  }

  int size() {return 3;}
};

struct Control
{
  float v, w; // linear velocity and angular velocity

  Control(): v(0.0), w(0.0) {}
  Control(float vv, float ww): v(v), w(ww) {}

  bool operator== (const Control& u)
  {
    float epsilon = 10e-6;
    if(u.v > v - epsilon && u.v < v + epsilon) // if v - epsilon < u.v < v + epsilon
    {
      if(u.w > w - epsilon && u.w < w + epsilon) // if w - epsilon < u.w < w + epsilon
      {
        return true;
      }
    }

    return false;
  }

  Eigen::VectorXd convert_to_eigen()
  {
    Eigen::VectorXd ans(this->size());
    ans << v, w;
    return ans;
  }

  int size() {return 2;}
};

#endif