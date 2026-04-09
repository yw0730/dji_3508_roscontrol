#ifndef DJI_M3508_SINGLE__SERIAL_UTILS_HPP_
#define DJI_M3508_SINGLE__SERIAL_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace dji_m3508_single
{

int open_serial_port(const std::string & device, int baudrate);
void close_serial_port(int fd);

ssize_t read_serial_bytes(int fd, uint8_t * buffer, std::size_t max_len);
bool write_serial_bytes(int fd, const uint8_t * data, std::size_t len);

}  // namespace dji_m3508_single

#endif  // DJI_M3508_SINGLE__SERIAL_UTILS_HPP_
