#include "rclcpp/rclcpp.hpp"
#include "canalystii_driver/canalystii.hpp"

namespace drivers
{
namespace can_driver
{

// std::string toHexString(const std::vector<uint8_t>& data) {
//     std::stringstream ss;
//     ss << std::hex << std::setfill('0');  // 设置填充字符为 '0'
//     for (const auto& byte : data) {
//         ss << std::setw(2) << static_cast<int>(byte);  // 输出每个字节
//     }
//     return ss.str();
// }
template <typename Container>
std::string toHexString(const Container& data) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');  // 设置填充字符为 '0'
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte);  // 输出每个字节
    }
    return ss.str();
}

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

// 这个函数没有用上，所以不必管其中header.stamp的一致性问题
int32_t CANalystii::receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &))
{
  int reclen = 0;
  VCI_CAN_OBJ rec[3000];  // 接收缓存，设为3000为佳。

  if (
    (reclen = VCI_Receive(vci_device_type_, vci_device_idx_, can_idx, rec, 3000, 20)) >
    0) {  // 调用接收函数，如果有数据，进行数据处理显示。
    for (int j = 0; j < reclen; j++) {
      auto msg = std::make_unique<can_msgs::msg::Frame>();
      // if (use_bus_time_) {
      //   msg->header.stamp = rclcpp::Time(static_cast<int64_t>(rec[j].TimeStamp * 100000U));
      // } else {
      //   msg->header.stamp = this->now();
      // }
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
  RCLCPP_DEBUG(logger, "send %d: %d", can_idx, frame.id);
  VCI_CAN_OBJ pkt;
  std::memset(&pkt, 0, sizeof(pkt));
  pkt.ID = frame.id;
  pkt.SendType = 1;  // 单次发送，不会自动重发
  pkt.RemoteFlag = frame.is_rtr ? 1 : 0;
  pkt.ExternFlag = frame.is_extended ? 1 : 0;
  pkt.DataLen = frame.dlc;
  std::copy(frame.data.begin(), frame.data.begin() + frame.dlc, pkt.Data);
  auto result = VCI_Transmit(vci_device_type_, vci_device_idx_, can_idx, &pkt, 1);
  if (result <= 0) {
    RCLCPP_INFO(logger, "send fail! id=%d", frame.id);
  }
  return result;
}


////// CANFromRecordFile
std::vector<uint8_t> stringToUint8Array(const std::string& str) {
    std::vector<uint8_t> result;
    std::istringstream iss(str);
    std::string byte;

    // 跳过第一个部分（'x|'）
    std::getline(iss, byte, '|');

    // 读取剩余的十六进制字节
    while (iss >> byte) {
        // 将十六进制字符串转换为整数
        int value = 0;
        std::istringstream byteStream(byte);
        byteStream >> std::hex >> value;  // 使用十六进制读取

        // 添加到结果数组
        result.push_back(static_cast<uint8_t>(value));
    }

    return result;
}

rclcpp::Logger logger2 = rclcpp::get_logger("CANFromRecordFile");

bool CANFromRecordFile::open()
{
  if (!stream_.is_open()) {
    stream_.open(file_);
  }
  if (!stream_.is_open()) {
    return false;
  }
  std::string line;
  getline(stream_, line); // 读取第一行,不需要第一行
  return true;
}

bool CANFromRecordFile::close()
{
  if (stream_.is_open())
    stream_.close();
  return true;
}

bool CANFromRecordFile::initCan(unsigned int can_idx, const BTR rate)
{
  return true;
}

// 这个函数没有用上，所以不必管其中header.stamp的一致性问题
int32_t CANFromRecordFile::receiveOnce(unsigned int can_idx, void (*callback)(can_msgs::msg::Frame &))
{
  return 0;
}

int32_t CANFromRecordFile::receiveOnceToBuffer(unsigned int can_idx, VCI_CAN_OBJ* pkts, unsigned int count) {
  // 读取时间一样的那些行 或者 每隔一定时间读一行
  // 当前线程暂停 100000 纳秒 = 0.1毫秒
  std::this_thread::sleep_for(std::chrono::nanoseconds(100000));

  // 检查是否到达文件末尾
  if (stream_.eof()) {
    RCLCPP_DEBUG(logger2, "Reached end of file");
    return 0;
  } else if (stream_.fail()) {
    RCLCPP_DEBUG(logger2, "Error reading file");
    return 0;
  }

  std::string line;
  getline(stream_, line); // 读取一行

  std::stringstream lineStream(line);
  std::string cell;
  std::vector<std::string> row;
  while (std::getline(lineStream, cell, ',')) {
    row.push_back(cell);
  }
  if (row.size() < 10) {
    RCLCPP_DEBUG(logger2, "incomplete row! maybe end of file");
    return 0;
  }

  int ID = std::stoi(row[5], nullptr, 16);
  int DataLen = std::stoi(row[8], nullptr, 16);
  std::vector<uint8_t> Data = stringToUint8Array(row[9]);
  if(Data.size() < DataLen) DataLen = Data.size();
  if (DataLen > 8) DataLen = 8;

  pkts[0].RemoteFlag = 0;
  pkts[0].ExternFlag = 0;
  pkts[0].DataLen = DataLen; // 第8字段
  pkts[0].ID = ID; // 第5字段
  // pkts[0].TimeStamp = 0; // 第1，2字段
  // pkts[0].Data = // 第9字段
  std::copy(Data.begin(), Data.begin()+DataLen, pkts[0].Data);
  return 1;
}

int32_t CANFromRecordFile::send(unsigned int can_idx, const can_msgs::msg::Frame & frame)
{
  std::string str = toHexString(frame.data);
  // RCLCPP_INFO(logger, "send %d: %s", frame.id, str.c_str());
  return 1;
}

}  // namespace can_driver
}  // namespace drivers
