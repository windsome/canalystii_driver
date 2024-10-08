#include "rclcpp/rclcpp.hpp"
#include "canalystii_driver/canalystii.hpp"

namespace drivers
{
namespace can_driver
{
rclcpp::Logger logger = rclcpp::get_logger("CANalystii");

bool CANalystii::open()
{
  DWORD result = VCI_OpenDevice(vci_device_type_, vci_device_idx_, 0);
  if (result == 1) {
    RCLCPP_INFO(logger, "VCI_OpenDevice OK.");
    return true;
  }
  RCLCPP_INFO(logger, "pause 2 seconds, maybe device is restarting.");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  result = VCI_OpenDevice(vci_device_type_, vci_device_idx_, 0);
  if (result == 1) {
    RCLCPP_INFO(logger, "VCI_OpenDevice OK at next try!");
    return true;
  }
  RCLCPP_ERROR(logger, "VCI_OpenDevice FAIL twice.");
  return false;
}

bool CANalystii::close()
{
  DWORD result = VCI_CloseDevice(vci_device_type_, vci_device_idx_);
  if (result == 1) {
    RCLCPP_INFO(logger, "VCI_CloseDevice OK.");
    return true;
  }
  RCLCPP_ERROR(logger, "VCI_CloseDevice FAIL.");
  return false;
}

bool CANalystii::initCan(unsigned int can_idx, const BTR rate)
{
  uint16_t btr_rate = static_cast<uint16_t>(rate);
  VCI_INIT_CONFIG conf;
  conf.AccCode = 0x80000008;
  conf.AccMask = 0xFFFFFFFF;
  conf.Filter = 1;  // receive all frames
  conf.Timing0 = btr_rate & 0xFF;
  conf.Timing1 = (btr_rate >> 8) & 0xFF;
  conf.Mode = 0;  // normal mode

  DWORD result1 = VCI_InitCAN(vci_device_type_, vci_device_idx_, can_idx, &conf);
  DWORD result2 = VCI_StartCAN(vci_device_type_, vci_device_idx_, can_idx);
  VCI_ClearBuffer(vci_device_type_, vci_device_idx_, can_idx);
  VCI_ClearBuffer(vci_device_type_, vci_device_idx_, can_idx);
  return (result1 == 1) && (result2 == 1);
}

int32_t CANalystii::receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &))
{
  int reclen = 0;
  VCI_CAN_OBJ rec[3000];  // 接收缓存，设为3000为佳。

  if (
    (reclen = VCI_Receive(vci_device_type_, vci_device_idx_, can_idx, rec, 3000, 20)) >
    0) {  // 调用接收函数，如果有数据，进行数据处理显示。
    for (int j = 0; j < reclen; j++) {
      auto msg = std::make_unique<can_msgs::msg::Frame>();
      msg->header.stamp = rclcpp::Time(static_cast<int64_t>(rec[j].TimeStamp * 100000U));
      msg->id = rec[j].ID;
      msg->is_rtr = rec[j].RemoteFlag == 1;
      msg->is_extended = rec[j].ExternFlag == 1;
      msg->is_error = false;
      msg->dlc = rec[j].DataLen;
      std::copy(rec[j].Data, rec[j].Data + msg->dlc, msg->data.begin());
      // std::move(data.begin(), data.end(), msg->data.begin());
      callback(*msg);
    }
  }
  return reclen;
}

int32_t CANalystii::receiveOnceToBuffer(unsigned int can_idx, VCI_CAN_OBJ* pkts, unsigned int count) {
  return VCI_Receive(vci_device_type_, vci_device_idx_, can_idx, pkts, count, 20);
}

int32_t CANalystii::send(unsigned int can_idx, const can_msgs::msg::Frame & frame)
{
  // TODO: (vincent.cheung.mcer@gmail.com) Not yet implemented.
  RCLCPP_INFO(logger, "send %d: %d", can_idx, frame.id);
  VCI_CAN_OBJ pkt;
  std::memset(&pkt, 0, sizeof(pkt));
  pkt.ID = frame.id;
  pkt.SendType = 1;  // 单次发送，不会自动重发
  pkt.RemoteFlag = frame.is_rtr ? 1 : 0;
  pkt.ExternFlag = frame.is_extended ? 1 : 0;
  pkt.DataLen = frame.dlc;
  std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, pkt.Data);
  return VCI_Transmit(vci_device_type_, vci_device_idx_, can_idx, &pkt, 1);
}

}  // namespace can_driver
}  // namespace drivers
