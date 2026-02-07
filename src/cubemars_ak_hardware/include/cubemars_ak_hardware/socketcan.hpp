#pragma once

#include <linux/can.h>

#include <optional>
#include <string>

namespace cubemars_ak_hardware
{

// Minimal SocketCAN wrapper for a non-blocking RAW CAN socket.
// - open(): creates socket, binds to interface (e.g., "can0"), sets non-blocking
// - recv(): returns a CAN frame if available, otherwise std::nullopt
// - send(): writes a CAN frame
class SocketCAN
{
public:
  SocketCAN() = default;
  ~SocketCAN();

  // Returns true on success. If error_msg is provided, it will contain a readable error.
  bool open(const std::string & ifname, std::string * error_msg = nullptr);

  void close();

  bool is_open() const { return fd_ >= 0; }

  // Non-blocking receive.
  std::optional<can_frame> recv(std::string * error_msg = nullptr);

  // Send a frame. Returns true on success.
  bool send(const can_frame & frame, std::string * error_msg = nullptr);

private:
  int fd_{-1};
};

}  // namespace cubemars_ak_hardware
