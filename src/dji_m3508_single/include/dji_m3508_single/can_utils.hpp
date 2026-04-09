#ifndef DJI_M3508_SINGLE__CAN_UTILS_HPP_
#define DJI_M3508_SINGLE__CAN_UTILS_HPP_

#include <linux/can.h>
#include <string>

namespace dji_m3508_single
{

bool configure_can_interface(const std::string & ifname, int bitrate);
int open_can_socket(const std::string & ifname);
bool set_socket_nonblocking(int fd);
bool write_can_frame(int fd, const can_frame & frame);
bool read_can_frame(int fd, can_frame & frame);

}  // namespace dji_m3508_single

#endif  // DJI_M3508_SINGLE__CAN_UTILS_HPP_
