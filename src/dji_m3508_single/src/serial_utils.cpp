#include "dji_m3508_single/serial_utils.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>

namespace dji_m3508_single
{

namespace
{

speed_t to_baud_constant(int baudrate)
{
  switch (baudrate) {
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
#ifdef B921600
      return B921600;
#else
      return 0;
#endif
    default:
      return 0;
  }
}

bool configure_port_raw_8n1(int fd, int baudrate)
{
  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }

  const speed_t speed = to_baud_constant(baudrate);
  if (speed == 0) {
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    return false;
  }

  // Non-blocking IO: return immediately when no bytes are available.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    return false;
  }

  return true;
}

}  // namespace

int open_serial_port(const std::string & device, int baudrate)
{
  const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }

  if (!configure_port_raw_8n1(fd, baudrate)) {
    close(fd);
    return -1;
  }

  tcflush(fd, TCIOFLUSH);
  return fd;
}

void close_serial_port(int fd)
{
  if (fd >= 0) {
    close(fd);
  }
}

ssize_t read_serial_bytes(int fd, uint8_t * buffer, std::size_t max_len)
{
  const ssize_t n = read(fd, buffer, max_len);
  if (n >= 0) {
    return n;
  }

  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return 0;
  }

  return -1;
}

bool write_serial_bytes(int fd, const uint8_t * data, std::size_t len)
{
  std::size_t written = 0;
  while (written < len) {
    const ssize_t n = write(fd, data + written, len - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }

    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      continue;
    }

    return false;
  }

  return true;
}

}  // namespace dji_m3508_single
