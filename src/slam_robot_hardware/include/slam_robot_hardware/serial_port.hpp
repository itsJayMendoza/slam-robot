// Minimal blocking line I/O on a POSIX serial port (raw 8N1), with a read timeout.
#pragma once
#include <string>

namespace slam_robot_hardware
{

class SerialPort
{
public:
  ~SerialPort();
  bool open(const std::string & device, int baud, std::string & error);
  void close();
  bool is_open() const {return fd_ >= 0;}
  void flush_input();
  bool write_line(const std::string & line);
  // Next non-empty line without the newline; false on timeout or error.
  bool read_line(std::string & line, int timeout_ms);

private:
  int fd_ = -1;
  std::string buffer_;
};

}  // namespace slam_robot_hardware
