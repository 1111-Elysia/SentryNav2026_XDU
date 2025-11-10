#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <string>
#include <vector>
#include <termios.h>

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();
  
  bool open(const std::string& port, int baudrate);
  void close();
  bool isOpen() const;
  
  int write(const uint8_t* data, size_t length);
  int read(uint8_t* buffer, size_t length);
  
private:
  int fd_;
  struct termios old_tio_;
  bool is_open_;
};

#endif
