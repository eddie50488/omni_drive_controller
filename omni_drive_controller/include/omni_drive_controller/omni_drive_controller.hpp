#ifndef OMNI_CONTROLLER_OMNIDIRECTIONAL_CONTROLLER_HPP_
#define OMNI_CONTROLLER_OMNIDIRECTIONAL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "omni_drive_controller/kinematics.hpp"
#include "omni_drive_controller/types.hpp"

namespace omni_drive_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class OmniDriveController : public controller_interface::ControllerInterface
  {
  public:
    OmniDriveController();
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    ~OmniDriveController();

  protected:
    struct WheelHandle
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
      std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };

    std::vector<std::string> wheel_names_;
    std::vector<WheelHandle> registered_wheel_handles_;
    RobotParams robot_params_{0.0505};
    bool use_stamped_vel_ = true;
    Kinematics omni_robot_kinematics_;

    // Timeout to consider cmd_vel commands old
    std::chrono::milliseconds cmd_vel_timeout_{1000};
    bool subscriber_is_active_ = false;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_subscriber_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_unstamped_subscriber_ = nullptr;
    rclcpp::Time previous_update_timestamp_{0};

  private:
    void velocityCommandStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel);
    void velocityCommandUnstampedCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  };

} // namespace omnidirectional_controllers

#endif // OMNI_CONTROLLER_OMNIDIRECTIONAL_CONTROLLER_HPP_
