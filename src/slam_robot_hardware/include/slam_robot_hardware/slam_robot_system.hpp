// ros2_control SystemInterface for the slam-robot ESP32 base (two wheels, velocity commands).
#pragma once
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "slam_robot_hardware/protocol.hpp"
#include "slam_robot_hardware/serial_port.hpp"

namespace slam_robot_hardware
{

class SlamRobotSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Wheel
  {
    std::string joint;
    double command = 0.0;    // rad/s
    double position = 0.0;   // rad
    double velocity = 0.0;   // rad/s
    long offset = 0;         // counts at activation
  };

  // Sends cmd and returns the first reply starting with `expect` (or "err"); '#' lines are logged.
  bool request(const std::string & cmd, const std::string & expect, std::string & reply);
  bool apply_state(const std::string & reply);
  hardware_interface::return_type link_failed(const std::string & what);

  std::string port_ = "/dev/esp32";
  int baud_ = 115200;
  double counts_per_rev_ = 49130.0;
  double max_cps_ = 50000.0;
  int timeout_ms_ = 60;
  int boot_wait_ms_ = 2000;

  SerialPort serial_;
  Wheel left_, right_;
  BaseState state_;
  bool state_fresh_ = false;     // the last write's reply already carries the state
  int failures_ = 0;             // consecutive missing replies
  int fault_ = 0;
  rclcpp::Time zero_since_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SlamRobotSystem");
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace slam_robot_hardware
