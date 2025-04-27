#include "imu_usb_driver/imu_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <boost/system/system_error.hpp>
#include <array>

ImuReader::ImuReader(const std::string &port, unsigned int baud)
: serial_(io_)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ImuReader"),
    "Opening serial port '%s' at %u baud", port.c_str(), baud);
  serial_.open(port);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
}

ImuReader::~ImuReader() {
  RCLCPP_INFO(rclcpp::get_logger("ImuReader"), "Closing serial port");
  if (serial_.is_open()) serial_.close();
}

bool ImuReader::readNext(TINSData &out) {
  // Читаем сразу весь пакет с IMU
  static constexpr size_t PACKET_SIZE = sizeof(TOutINSPacket);
  std::array<uint8_t, PACKET_SIZE> buf;
  try {
    // блокируем до получения полного пакета
    boost::asio::read(serial_, boost::asio::buffer(buf.data(), buf.size()));
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ImuReader"),
      "Serial read error: %s", e.what());
    return false;
  }
  // передаем каждый байт в парсер
  for (auto b : buf) {
    ProcessReceivedByteFromUART(b);
  }
  // проверяем, появился ли новый пакет
  if (INSPacketsCounter > last_count_) {
    // последний добавленный пакет хранится по индексу (INSPacketsCounter-1)%SIZE
    uint32_t idx = (INSPacketsCounter - 1) % INS_PACKETS_LIST_SIZE;
    RCLCPP_DEBUG(
      rclcpp::get_logger("ImuReader"),
      "New packet #%u in slot %u", INSPacketsCounter, idx);
    GetINSDataFromOutINSPacket(&out, &INSPacketsList[idx]);
    last_count_ = INSPacketsCounter;
    return true;
  }
  return false;
}
