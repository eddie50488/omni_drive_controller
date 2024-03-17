#include <cmath>
#include <memory>

#include "omni_drive_controller/kinematics.hpp"
#include "omni_drive_controller/types.hpp"

namespace omni_drive_controller
{

  Kinematics::Kinematics(RobotParams robot_params)
      : robot_params_(robot_params)
  {
    this->initializeParams();
  }

  Kinematics::Kinematics()
  {
    this->initializeParams();
  }

  std::vector<double> Kinematics::getWheelsAngularVelocities(RobotVelocity vel)
  {
    double vx = vel.vx;
    double vy = vel.vy;
    double wl = vel.omega;
    double forward = 1.0;
    double backward = 0.0;
    double idle = 2.0;

    if (vx > 0)
    {
      angular_vel_vec_[0] = forward;
      angular_vel_vec_[1] = idle;
      angular_vel_vec_[2] = idle;
      angular_vel_vec_[3] = backward;
    }

    if (vx < 0)
    {
      angular_vel_vec_[0] = backward;
      angular_vel_vec_[1] = idle;
      angular_vel_vec_[2] = idle;
      angular_vel_vec_[3] = forward;
    }

    if (vy > 0)
    {
      angular_vel_vec_[0] = idle;
      angular_vel_vec_[1] = forward;
      angular_vel_vec_[2] = backward;
      angular_vel_vec_[3] = idle;
    }

    if (vy < 0)
    {
      angular_vel_vec_[0] = idle;
      angular_vel_vec_[1] = backward;
      angular_vel_vec_[2] = forward;
      angular_vel_vec_[3] = idle;
    }

    if (wl > 0)
    {
      angular_vel_vec_[0] = forward;
      angular_vel_vec_[1] = forward;
      angular_vel_vec_[2] = forward;
      angular_vel_vec_[3] = forward;
    }

    if (wl < 0)
    {
      angular_vel_vec_[0] = backward;
      angular_vel_vec_[1] = backward;
      angular_vel_vec_[2] = backward;
      angular_vel_vec_[3] = backward;
    }

    if (wl == 0 && vx == 0 && vy == 0)
    {
      angular_vel_vec_[0] = idle;
      angular_vel_vec_[1] = idle;
      angular_vel_vec_[2] = idle;
      angular_vel_vec_[3] = idle;
    }

    return angular_vel_vec_;
  }

  void Kinematics::setRobotParams(RobotParams robot_params)
  {
    this->robot_params_ = robot_params;
    this->initializeParams();
  }

  void Kinematics::initializeParams()
  {
    angular_vel_vec_.reserve(OMNI_ROBOT_WHEELS);
    angular_vel_vec_ = {4, 4, 4, 4};
  }

  Kinematics::~Kinematics() {}

} // namespace omnidirectional_controllers
