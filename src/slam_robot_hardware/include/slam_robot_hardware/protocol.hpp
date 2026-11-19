// Text protocol of the ESP32 base firmware (see firmware/src/base/main.cpp). No ROS, no I/O here.
#pragma once
#include <optional>
#include <string>

namespace slam_robot_hardware
{

struct BaseState
{
  long left = 0;           // encoder counts, forward positive
  long right = 0;
  double left_cps = 0.0;   // measured counts per second
  double right_cps = 0.0;
  int fault = 0;           // latched fault bits, 0 = none
};

// Parses "e <left> <right> <left cps> <right cps> <fault>"; nullopt for anything else.
std::optional<BaseState> parse_state(const std::string & line);

// "v <l> <r>" with speeds rounded to whole counts/s and clamped to +/-max_cps.
std::string velocity_command(double left_cps, double right_cps, double max_cps);

// Human-readable fault bits, e.g. "left stalled, right overspeed".
std::string fault_text(int fault);

}  // namespace slam_robot_hardware
