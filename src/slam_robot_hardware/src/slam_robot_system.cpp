#include "slam_robot_hardware/slam_robot_system.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace slam_robot_hardware
{

namespace
{
constexpr int kMaxFailures = 10;          // consecutive lost replies before giving up (0.2 s at 50 Hz)
constexpr double kFaultClearAfterS = 1.0; // a latched fault is cleared after this long at zero command

double param(const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : std::stod(it->second);
}
}  // namespace

SlamRobotSystem::CallbackReturn SlamRobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  auto port = info.hardware_parameters.find("port");
  if (port != info.hardware_parameters.end()) {port_ = port->second;}
  baud_ = static_cast<int>(param(info, "baud", baud_));
  counts_per_rev_ = param(info, "counts_per_rev", counts_per_rev_);
  max_cps_ = param(info, "max_cps", max_cps_);
  timeout_ms_ = static_cast<int>(param(info, "timeout_ms", timeout_ms_));
  boot_wait_ms_ = static_cast<int>(param(info, "boot_wait_ms", boot_wait_ms_));

  if (info.joints.size() != 2) {
    RCLCPP_FATAL(logger_, "expected 2 wheel joints, got %zu", info.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & j : info.joints) {
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger_, "joint %s: needs exactly one velocity command interface", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (j.name.find("left") != std::string::npos) {
      left_.joint = j.name;
    } else if (j.name.find("right") != std::string::npos) {
      right_.joint = j.name;
    }
  }
  if (left_.joint.empty() || right_.joint.empty()) {
    RCLCPP_FATAL(logger_, "wheel joint names must contain 'left' and 'right'");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SlamRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto * w : {&left_, &right_}) {
    out.emplace_back(w->joint, hardware_interface::HW_IF_POSITION, &w->position);
    out.emplace_back(w->joint, hardware_interface::HW_IF_VELOCITY, &w->velocity);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> SlamRobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto * w : {&left_, &right_}) {
    out.emplace_back(w->joint, hardware_interface::HW_IF_VELOCITY, &w->command);
  }
  return out;
}

SlamRobotSystem::CallbackReturn SlamRobotSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  if (!serial_.open(port_, baud_, error)) {
    RCLCPP_ERROR(logger_, "cannot open %s (is the ESP32 plugged in? dialout group?)", error.c_str());
    return CallbackReturn::ERROR;
  }
  // Opening the port can restart the ESP32; let it boot, then drop the boot messages.
  std::this_thread::sleep_for(std::chrono::milliseconds(boot_wait_ms_));
  serial_.flush_input();
  std::string reply;
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (request("ver", "ver ", reply) && reply.rfind("ver slam-robot-base", 0) == 0) {
      RCLCPP_INFO(logger_, "connected to %s on %s", reply.substr(4).c_str(), port_.c_str());
      return CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    serial_.flush_input();
  }
  RCLCPP_ERROR(logger_, "no slam-robot-base firmware answering on %s", port_.c_str());
  serial_.close();
  return CallbackReturn::ERROR;
}

SlamRobotSystem::CallbackReturn SlamRobotSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  serial_.close();
  return CallbackReturn::SUCCESS;
}

SlamRobotSystem::CallbackReturn SlamRobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::string reply;
  if (!request("x", "ok", reply) || !request("e", "e ", reply) || !apply_state(reply)) {
    RCLCPP_ERROR(logger_, "base not answering on activation");
    return CallbackReturn::ERROR;
  }
  left_.offset = state_.left;
  right_.offset = state_.right;
  for (auto * w : {&left_, &right_}) {
    w->command = w->position = w->velocity = 0.0;
  }
  state_fresh_ = false;
  failures_ = 0;
  fault_ = 0;
  zero_since_ = clock_.now();
  RCLCPP_INFO(logger_, "base active (%.0f counts per wheel turn)", counts_per_rev_);
  return CallbackReturn::SUCCESS;
}

SlamRobotSystem::CallbackReturn SlamRobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::string reply;
  request("v 0 0", "e ", reply);
  request("x", "ok", reply);
  return CallbackReturn::SUCCESS;
}

bool SlamRobotSystem::request(const std::string & cmd, const std::string & expect, std::string & reply)
{
  if (!serial_.write_line(cmd)) {return false;}
  std::string line;
  for (int i = 0; i < 8; ++i) {
    if (!serial_.read_line(line, timeout_ms_)) {return false;}
    if (line[0] == '#') {
      RCLCPP_WARN(logger_, "firmware: %s", line.c_str() + 1);
      continue;
    }
    if (line.rfind(expect, 0) == 0 || line.rfind("err", 0) == 0) {
      reply = line;
      return true;
    }
    // anything else is a stray line (e.g. a cut boot banner): skip it
  }
  return false;
}

bool SlamRobotSystem::apply_state(const std::string & reply)
{
  auto s = parse_state(reply);
  if (!s) {return false;}
  state_ = *s;
  return true;
}

hardware_interface::return_type SlamRobotSystem::link_failed(const std::string & what)
{
  ++failures_;
  RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "no reply to %s (%d in a row)", what.c_str(), failures_);
  if (failures_ >= kMaxFailures) {
    RCLCPP_ERROR(logger_, "lost the ESP32 base on %s", port_.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SlamRobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!state_fresh_) {
    std::string reply;
    if (!request("e", "e ", reply) || !apply_state(reply)) {return link_failed("e");}
  }
  state_fresh_ = false;
  failures_ = 0;
  const double rad_per_count = 2.0 * M_PI / counts_per_rev_;
  left_.position = (state_.left - left_.offset) * rad_per_count;
  right_.position = (state_.right - right_.offset) * rad_per_count;
  left_.velocity = state_.left_cps * rad_per_count;
  right_.velocity = state_.right_cps * rad_per_count;

  if (state_.fault != fault_) {
    if (state_.fault) {
      RCLCPP_ERROR(logger_, "base fault: %s; motors stopped until the commands are zero for %.0f s",
        fault_text(state_.fault).c_str(), kFaultClearAfterS);
    }
    fault_ = state_.fault;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SlamRobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double cps_per_rad = counts_per_rev_ / (2.0 * M_PI);
  const bool zero = left_.command == 0.0 && right_.command == 0.0;
  if (!zero) {zero_since_ = clock_.now();}
  std::string reply;
  if (fault_ && zero && (clock_.now() - zero_since_).seconds() > kFaultClearAfterS) {
    if (request("x", "ok", reply)) {
      RCLCPP_INFO(logger_, "fault cleared");
      fault_ = 0;
    }
  }
  const std::string cmd = velocity_command(left_.command * cps_per_rad, right_.command * cps_per_rad, max_cps_);
  if (!request(cmd, "e ", reply) || !apply_state(reply)) {return link_failed("v");}
  state_fresh_ = true;   // this reply is the newest state; the next read uses it
  return hardware_interface::return_type::OK;
}

}  // namespace slam_robot_hardware

PLUGINLIB_EXPORT_CLASS(slam_robot_hardware::SlamRobotSystem, hardware_interface::SystemInterface)
