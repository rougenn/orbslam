#pragma once

#include <string>
#include <boost/asio.hpp>
extern "C" {
  #include "INS_packets.h"
}

/// Обёртка для последовательного порта и разбора байт‑пакетов IMU
class ImuReader {
public:
  ImuReader(const std::string &port, unsigned int baud);
  ~ImuReader();
  /// Читает следующий полный пакет. true если пакет готов.
  bool readNext(TINSData &out);

private:
  boost::asio::io_service    io_;
  boost::asio::serial_port    serial_;
  uint32_t                    last_count_ = 0;
};
