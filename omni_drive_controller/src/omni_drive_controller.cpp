#include "omni_drive_controller/omni_drive_controller.hpp"

#include <chrono> // NOLINT
#include <cmath>
#include <exception>
#include <iostream>


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace
{
  constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
  constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
}

namespace omni_drive_controller
{

  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using hardware_interface::HW_IF_VELOCITY;
  using lifecycle_msgs::msg::State;
  using std::placeholders::_1;

  // initializes the cmd_vel_ member variable with a shared pointer to
  // a new instance of geometry_msgs::msg::TwistStamped.
  OmniDriveController::OmniDriveController()
      : controller_interface::ControllerInterface(), cmd_vel_(std::make_shared<geometry_msgs::msg::TwistStamped>()) {}

  CallbackReturn OmniDriveController::on_init()
  {

    this->node_ = this->get_node();

    try
    {
      auto_declare<std::vector<std::string>>("wheel_names", std::vector<std::string>());
      auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
      auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration OmniDriveController::command_interface_configuration() const
  {
    std::vector<std::string> conf_names;

    for (const auto &joint_name : wheel_names_)
    {
      conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration OmniDriveController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (const auto &joint_name : wheel_names_)
    {
      conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  CallbackReturn OmniDriveController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    auto logger = node_->get_logger();
    RCLCPP_DEBUG(logger,
                 "Called on_configure. Previous state was %s",
                 previous_state.label().c_str());

    // update parameters
    wheel_names_ = node_->get_parameter("wheel_names").as_string_array();
    omni_robot_kinematics_.setRobotParams(robot_params_);
    cmd_vel_timeout_ = std::chrono::milliseconds{
        static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
    use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

    // initialize command subscriber
    if (use_stamped_vel_)
    {
      vel_cmd_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          DEFAULT_COMMAND_TOPIC,
          rclcpp::SystemDefaultsQoS(),
          std::bind(&OmniDriveController::velocityCommandStampedCallback, this, _1));
    }
    else
    {
      vel_cmd_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          DEFAULT_COMMAND_UNSTAMPED_TOPIC,
          rclcpp::SystemDefaultsQoS(),
          std::bind(&OmniDriveController::velocityCommandUnstampedCallback, this, _1));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn OmniDriveController::on_activate(
      const rclcpp_lifecycle::State &previous_state)
  {
    auto logger = node_->get_logger();
    RCLCPP_DEBUG(logger,
                 "Called on_activate. Previous state was %s",
                 previous_state.label().c_str());

    registered_wheel_handles_.reserve(wheel_names_.size());
    for (const auto &wheel_name : wheel_names_)
    {
      std::string interface_name = wheel_name + "/" + HW_IF_VELOCITY;

      const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                                             [&interface_name](const auto &interface)
                                             {
                                               return interface.get_name() == interface_name;
                                             });

      const auto command_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                                               [&interface_name](const auto &interface)
                                               {
                                                 return interface.get_name() == interface_name;
                                               });

      registered_wheel_handles_.emplace_back(
          WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

      RCLCPP_INFO(logger, "Got command interface: %s", command_handle->get_name().c_str());
      RCLCPP_INFO(logger, "Got state interface: %s", state_handle->get_name().c_str());
    }
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
    previous_update_timestamp_ = node_->get_clock()->now();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn OmniDriveController::on_deactivate(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "Called on_deactivate. Previous state was %s",
                 previous_state.label().c_str());
    subscriber_is_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  void OmniDriveController::velocityCommandStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }
  if ((cmd_vel->header.stamp.sec == 0) && (cmd_vel->header.stamp.nanosec == 0)) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "Received TwistStamped with zero timestamp, setting it to current "
      "time, this message will only be shown once");
    cmd_vel->header.stamp = node_->get_clock()->now();
  }

  this->cmd_vel_ = std::move(cmd_vel);
}

  void OmniDriveController::velocityCommandUnstampedCallback(
    const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
    if (!subscriber_is_active_) {
      RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
      return;
    }
    this->cmd_vel_->twist = *cmd_vel;
    this->cmd_vel_->header.stamp = node_->get_clock()->now();
  }

  controller_interface::return_type OmniDriveController::update(
      const rclcpp::Time &time,
      const rclcpp::Duration & /*period*/)
  {
    auto logger = node_->get_logger();
    const auto current_time = time;
    const auto dt = current_time - cmd_vel_->header.stamp;
    auto cmd_time = cmd_vel_->header.stamp;
    printf("cmd_time: %d seconds, %d nanoseconds\n", cmd_time.sec, cmd_time.nanosec);

    double current_times = current_time.seconds();
    printf("current_time: %.2f\n", current_times);
    
    double dt_seconds = dt.seconds();
    printf("dt_seconds: %.2f\n", dt_seconds);

    // Get body velocity:
    RobotVelocity body_vel_setpoint;
    body_vel_setpoint.vx = cmd_vel_->twist.linear.x;
    body_vel_setpoint.vy = cmd_vel_->twist.linear.y;
    body_vel_setpoint.omega = cmd_vel_->twist.angular.z;

    // double dt_seconds = dt.seconds();
    
    // printf("hello: %.2f\n", dt);

    if (dt > cmd_vel_timeout_)
    {
      // printf("dt: %.2f\n", dt);
      cmd_vel_->twist.linear.x = 0.0;
      cmd_vel_->twist.linear.y = 0.0;
      cmd_vel_->twist.angular.z = 0.0;
    }
    
    // if (dt > cmd_vel_timeout_)
    // {
    //   body_vel_setpoint.vx = 0.0;
    //   body_vel_setpoint.vy = 0.0;
    //   body_vel_setpoint.omega = 0.0;
    // }


    // Calculate wheel velocities:
    std::vector<double> wheels_angular_velocity;
    wheels_angular_velocity = omni_robot_kinematics_.getWheelsAngularVelocities(body_vel_setpoint);

    // Set wheel velocities:
    registered_wheel_handles_[0].velocity_command.get().set_value(wheels_angular_velocity.at(0)); // Triangle
    registered_wheel_handles_[1].velocity_command.get().set_value(wheels_angular_velocity.at(1)); // Plus
    registered_wheel_handles_[2].velocity_command.get().set_value(wheels_angular_velocity.at(2)); // Circle
    registered_wheel_handles_[3].velocity_command.get().set_value(wheels_angular_velocity.at(3)); // Square

    return controller_interface::return_type::OK;
  }

  OmniDriveController::~OmniDriveController() {}

} // namespace omni_drive_controller

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerInterface)


#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(omni_drive_controller::OmniDriveController, controller_interface::ControllerInterface)