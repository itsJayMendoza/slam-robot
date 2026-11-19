#include "slam_robot_hardware/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace slam_robot_hardware
{

std::optional<BaseState> parse_state(const std::string & line)
{
  std::istringstream in(line);
  std::string tag;
  BaseState s;
  if (!(in >> tag) || tag != "e") {return std::nullopt;}
  if (!(in >> s.left >> s.right >> s.left_cps >> s.right_cps >> s.fault)) {return std::nullopt;}
  std::string extra;
  if (in >> extra) {return std::nullopt;}
  return s;
}

std::string velocity_command(double left_cps, double right_cps, double max_cps)
{
  auto clamp = [max_cps](double v) {
      if (!std::isfinite(v)) {return 0L;}
      return std::lround(std::clamp(v, -max_cps, max_cps));
    };
  return "v " + std::to_string(clamp(left_cps)) + " " + std::to_string(clamp(right_cps));
}

std::string fault_text(int fault)
{
  static const char * names[] = {"left stalled", "right stalled", "left turning the wrong way",
    "right turning the wrong way", "left overspeed", "right overspeed"};
  std::string out;
  for (int bit = 0; bit < 6; ++bit) {
    if (fault & (1 << bit)) {out += (out.empty() ? "" : ", ") + std::string(names[bit]);}
  }
  return out.empty() ? "none" : out;
}

}  // namespace slam_robot_hardware
