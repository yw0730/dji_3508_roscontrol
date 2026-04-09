#include "dji_m3508_single/can_utils.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

#include <linux/can/raw.h>

namespace dji_m3508_single
{

bool configure_can_interface(const std::string & ifname, int bitrate)
{
  // 通过 ip 命令配置 CAN 口波特率，失败时由上层决定是否继续。
  std::ostringstream cmd;
  cmd << "ip link set " << ifname << " down && "
      << "ip link set " << ifname << " type can bitrate " << bitrate << " && "
      << "ip link set " << ifname << " up";
  return std::system(cmd.str().c_str()) == 0;
}

int open_can_socket(const std::string & ifname)
{
  const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return -1;
  }

  ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    close(fd);
    return -1;
  }

  sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

bool set_socket_nonblocking(int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool write_can_frame(int fd, const can_frame & frame)
{
  const ssize_t nbytes = write(fd, &frame, sizeof(can_frame));
  return nbytes == static_cast<ssize_t>(sizeof(can_frame));
}

bool read_can_frame(int fd, can_frame & frame)
{
  const ssize_t nbytes = read(fd, &frame, sizeof(can_frame));
  if (nbytes == static_cast<ssize_t>(sizeof(can_frame))) {
    return true;
  }

  if (nbytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  }

  return false;
}

}  // namespace dji_m3508_single
