#ifndef CANALYSTII_H
#define CANALYSTII_H

#include "controlcan.h"
#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"

#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iostream>

namespace drivers
{
namespace can_driver
{

enum class BTR : uint16_t {  // 使用 uint8_t 作为底层类型
  R10K = 0x1C31,
  R20K = 0x1C18,
  R40K = 0xFF87,
  R50K = 0x1C09,
  R80K = 0xFF83,
  R100K = 0x1C04,
  R125K = 0x1C03,
  R200K = 0xFA81,
  R250K = 0x1C01,
  R400K = 0xFA80,
  R500K = 0x1C00,
  R666K = 0xB680,
  R800K = 0x1600,
  R1000K = 0x1400,
  R33_33K = 0x6F09,
  R66_66K = 0x6F04,
  R83_33K = 0x6F03,
};

class BaseCAN
{
public:
  BaseCAN(){};
  virtual ~BaseCAN() = 0;
  virtual bool open() = 0;   // 打开设备
  virtual bool close() = 0;  // 关闭设备

  // init CAN interface:port 1 or port 2
  virtual bool initCan(unsigned int can_idx = 0, const BTR rate = BTR::R500K) = 0;
  // receive CAN frame from CAN bus
  virtual int32_t receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &)) = 0;
  virtual int32_t receiveOnceToBuffer(unsigned int can_idx, VCI_CAN_OBJ * pkts, unsigned int count) = 0;
  // send CAN frame from CAN bus
  virtual int32_t send(unsigned int can_idx, const can_msgs::msg::Frame & frame) = 0;
};

// CANalyst-ii is can-usb device class, which offers
// start_device, close_device, init_can_interface,
// receive_can_frame, send_can_frame methods. By
// using the provided libcontrol.so
// Simple usage:
// CANalystii can_device;
// can_device.start_device();
// can_device.init_can_interface();
// can_device.receive_can_frame();//listening from CAN bus
// can_device.send_can_frame();//send to CAN bus
class CANalystii : public BaseCAN
{
public:
  CANalystii(int device_index = 0, int device_type = VCI_USBCAN2, bool use_bus_time = false)
  : vci_device_type_(device_type), vci_device_idx_(device_index), use_bus_time_(use_bus_time) {
      // open();
    };
  ~CANalystii() { close(); };
  bool open();   // 打开设备
  bool close();  // 关闭设备

  // init CAN interface:port 1 or port 2
  bool initCan(unsigned int can_idx = 0, const BTR rate = BTR::R500K);
  // receive CAN frame from CAN bus
  int32_t receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &));
  int32_t receiveOnceToBuffer(unsigned int can_idx, VCI_CAN_OBJ * pkts, unsigned int count);
  // send CAN frame from CAN bus
  int32_t send(unsigned int can_idx, const can_msgs::msg::Frame & frame);

private:
  unsigned int vci_device_type_;
  unsigned int vci_device_idx_;
  bool use_bus_time_{false};
};

class CANFromRecordFile : public BaseCAN
{
public:
  CANFromRecordFile(std::string file, bool use_bus_time = false)
  : file_(file), use_bus_time_(use_bus_time), stream_(file) {
    // open();
  };
  ~CANFromRecordFile() { close(); };
  bool open();   // 打开设备
  bool close();  // 关闭设备

  // init CAN interface:port 1 or port 2
  bool initCan(unsigned int can_idx = 0, const BTR rate = BTR::R500K);
  // receive CAN frame from CAN bus
  int32_t receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &));
  int32_t receiveOnceToBuffer(unsigned int can_idx, VCI_CAN_OBJ * pkts, unsigned int count);
  // send CAN frame from CAN bus
  int32_t send(unsigned int can_idx, const can_msgs::msg::Frame & frame);

private:
  std::string file_;
  bool use_bus_time_{false};
  std::ifstream stream_;
};


}  // namespace can_driver
}  // namespace drivers

#endif