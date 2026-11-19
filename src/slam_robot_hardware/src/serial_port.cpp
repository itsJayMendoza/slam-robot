#include "slam_robot_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace slam_robot_hardware
{

namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 230400: return B230400;
    default: return B115200;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & device, int baud, std::string & error)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = device + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error = device + ": tcgetattr: " + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);                  // no echo, no line editing, no flow control
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  cfsetispeed(&tio, to_speed(baud));
  cfsetospeed(&tio, to_speed(baud));
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = device + ": tcsetattr: " + std::strerror(errno);
    close();
    return false;
  }
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
  buffer_.clear();
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {tcflush(fd_, TCIFLUSH);}
  buffer_.clear();
}

bool SerialPort::write_line(const std::string & line)
{
  if (fd_ < 0) {return false;}
  std::string out = line + "\n";
  size_t done = 0;
  while (done < out.size()) {
    ssize_t n = ::write(fd_, out.data() + done, out.size() - done);
    if (n > 0) {
      done += static_cast<size_t>(n);
    } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    } else {
      pollfd p{fd_, POLLOUT, 0};
      if (poll(&p, 1, 50) <= 0) {return false;}
    }
  }
  return true;
}

bool SerialPort::read_line(std::string & line, int timeout_ms)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (fd_ >= 0) {
    size_t nl;
    while ((nl = buffer_.find('\n')) != std::string::npos) {
      line = buffer_.substr(0, nl);
      buffer_.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (!line.empty()) {return true;}
    }
    auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count();
    if (left <= 0) {return false;}
    pollfd p{fd_, POLLIN, 0};
    int r = poll(&p, 1, static_cast<int>(left));
    if (r <= 0) {return false;}
    char chunk[256];
    ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      buffer_.append(chunk, static_cast<size_t>(n));
      if (buffer_.size() > 4096) {buffer_.erase(0, buffer_.size() - 1024);}   // runaway input
    } else if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    }
  }
  return false;
}

}  // namespace slam_robot_hardware
