#include "serial_comm/serial_port.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

SerialPort::SerialPort() : fd_(-1), is_open_(false) {}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string& port, int baudrate)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) return false;

  struct termios tio;
  tcgetattr(fd_, &old_tio_);
  memset(&tio, 0, sizeof(tio));
  
  tio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  
  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &tio);
  
  is_open_ = true;
  return true;
}

void SerialPort::close()
{
  if (is_open_) {
    tcsetattr(fd_, TCSANOW, &old_tio_);
    ::close(fd_);
    is_open_ = false;
  }
}

bool SerialPort::isOpen() const
{
  return is_open_;
}

int SerialPort::write(const uint8_t* data, size_t length)
{
  if (!is_open_) return -1;
  return ::write(fd_, data, length);
}

int SerialPort::read(uint8_t* buffer, size_t length)
{
  if (!is_open_) return -1;
  return ::read(fd_, buffer, length);
}
