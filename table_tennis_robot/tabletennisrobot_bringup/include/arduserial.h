#ifndef ARDUSERIAL_H
#define ARDUSERIAL_H
#include "serial/serial.h"
#include <ros/ros.h>

class ArduSerial{
public:
  ArduSerial(std::string port, size_t length);
  ~ArduSerial();
  void init();
  void read(uint8_t *data, size_t size);
  void write(uint8_t *data, size_t size);
  void read_dick(uint8_t *data, size_t size);

private:

public:
  uint8_t readdata[2];
private:
  serial::Serial _ser;
  std::string _port;
  size_t _length;
  

  std::string _input;
  std::string _read;
  int data_packet_start;
};

#endif // ARDUSERIAL_H