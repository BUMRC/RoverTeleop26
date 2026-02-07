#include "cubemars_ak_hardware/socketcan.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fcntl.h>

namespace cubemars_ak_hardware
{

SocketCAN::~SocketCAN()
{
  close();
}

bool SocketCAN::open(const std::string & ifname, std::string * error_msg)
{
  close();

  // Create a RAW CAN socket.
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    if (error_msg) {
      *error_msg = "socket(PF_CAN) failed: " + std::string(std::strerror(errno));
    }
    return false;
  }

  // Make it non-blocking so read() doesn't stall the real-time control loop.
  int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    if (error_msg) {
      *error_msg = "fcntl(O_NONBLOCK) failed: " + std::string(std::strerror(errno));
    }
    close();
    return false;
  }

  // Optional: don't receive frames we transmit ourselves.
  int recv_own_msgs = 0;
  (void)::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

  // Resolve interface name (e.g. "can0") -> ifindex
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());

  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    if (error_msg) {
      *error_msg = "ioctl(SIOCGIFINDEX) failed for " + ifname + ": " + std::string(std::strerror(errno));
    }
    close();
    return false;
  }

  // Bind socket to the CAN interface.
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    if (error_msg) {
      *error_msg = "bind(AF_CAN) failed: " + std::string(std::strerror(errno));
    }
    close();
    return false;
  }

  return true;
}

void SocketCAN::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::optional<can_frame> SocketCAN::recv(std::string * error_msg)
{
  if (fd_ < 0) {
    if (error_msg) {
      *error_msg = "recv() called while socket is closed";
    }
    return std::nullopt;
  }

  can_frame frame;
  const ssize_t n = ::read(fd_, &frame, sizeof(frame));

  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return std::nullopt;  // no data available right now
    }
    if (error_msg) {
      *error_msg = "read(can) failed: " + std::string(std::strerror(errno));
    }
    return std::nullopt;
  }

  if (n != static_cast<ssize_t>(sizeof(frame))) {
    // Partial read shouldn't happen for can_frame, but handle defensively.
    if (error_msg) {
      *error_msg = "read(can) returned unexpected size: " + std::to_string(n);
    }
    return std::nullopt;
  }

  return frame;
}

bool SocketCAN::send(const can_frame & frame, std::string * error_msg)
{
  if (fd_ < 0) {
    if (error_msg) {
      *error_msg = "send() called while socket is closed";
    }
    return false;
  }

  const ssize_t n = ::write(fd_, &frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    if (error_msg) {
      *error_msg = "write(can) failed: " + std::string(std::strerror(errno));
    }
    return false;
  }

  return true;
}

}  // namespace cubemars_ak_hardware
