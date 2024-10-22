// Copyright 2021 LeoDrive, Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "canalystii_driver/can_bridge_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;
using namespace std::chrono_literals;  // 允许使用 ms 等后缀

namespace drivers
{
namespace can_driver
{

CanBridgeNode::CanBridgeNode(const rclcpp::NodeOptions & options)
: lc::LifecycleNode("can_bridge_node", options)
{
  RCLCPP_DEBUG(get_logger(), "CanBridgeNode()");
  get_params();
}

CanBridgeNode::~CanBridgeNode()
{
  RCLCPP_DEBUG(get_logger(), "~CanBridgeNode()");
  if (receiver_thread_ && receiver_thread_->joinable()) {
    receiver_thread_->join();  // 确保线程在析构时被正确关闭
  }
  // frames_pub_.reset();
  // frames_sub_.reset();
  // driver_->close();
  RCLCPP_DEBUG(get_logger(), "~CanBridgeNode() finish");
  // if (m_owned_ctx) {
  //   m_owned_ctx->waitForExit();
  // }
}

LNI::CallbackReturn CanBridgeNode::on_configure(const lc::State & state)
{
  (void)state;
  try {
    if (test_csv_ && csv_path_ != "") {
      RCLCPP_INFO(get_logger(), "start CANFromRecordFile.");
      driver_ = std::make_unique<CANFromRecordFile>(csv_path_, use_bus_time_);
    } else {
      RCLCPP_INFO(get_logger(), "start CANalystii.");
      driver_ = std::make_unique<CANalystii>(device_idx_);
    }
    // 接收到創芯轉換器消息，發送到autoware：from_can_bus
    frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 500);
    // 接收到autoware：to_can_bus消息，發往創芯轉換器
    frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "to_can_bus", 500, std::bind(&CanBridgeNode::on_frame, this, std::placeholders::_1));
    // 啓動接收創芯轉換器消息的線程
    receiver_thread_ = std::make_unique<std::thread>(&CanBridgeNode::receive, this);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error init: %s", ex.what());
    return LNI::CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "Can port successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanBridgeNode::on_activate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_activate();
  RCLCPP_DEBUG(get_logger(), "Can bridge activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanBridgeNode::on_deactivate(const lc::State & state)
{
  (void)state;
  frames_pub_->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "RCLCPP_ bridge deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanBridgeNode::on_cleanup(const lc::State & state)
{
  (void)state;
  try {
    RCLCPP_DEBUG(get_logger(), "Can bridge clean up start.");
    driver_->close();
    frames_pub_.reset();
    frames_sub_.reset();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error during shutdown: %s", ex.what());
  }
  RCLCPP_DEBUG(get_logger(), "Can bridge cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanBridgeNode::on_shutdown(const lc::State & state)
{
  (void)state;
  // try {
  //   driver_->close();
  //   RCLCPP_DEBUG(get_logger(), "frames_pub_.reset()");
  //   frames_pub_.reset();
  //   frames_sub_.reset();
  //   RCLCPP_DEBUG(get_logger(), "Can bridge shutting down.");
  // } catch (const std::exception & ex) {
  //   RCLCPP_ERROR(get_logger(), "Error during shutdown: %s", ex.what());
  // }
  RCLCPP_DEBUG(get_logger(), "Can bridge shutting down.");

  return LNI::CallbackReturn::SUCCESS;
}

void CanBridgeNode::get_params()
{
  try {
    frame_id_ = declare_parameter<std::string>("frame_id", "can");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The frame_id provided was invalid");
  }

  try {
    device_idx_ = static_cast<uint32_t>(declare_parameter<int>("device_idx", 0));
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device_idx provided was invalid");
  }

  try {
    can_idx_ = static_cast<uint32_t>(declare_parameter<int>("can_idx", 0));
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The can_idx provided was invalid");
  }

  try {
    can_baud_rate_ = static_cast<uint32_t>(declare_parameter<int>("can_baud_rate", 0x1C00));
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The can_baud_rate provided was invalid");
  }

  try {
    use_bus_time_ = declare_parameter<bool>("use_bus_time", false);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The use_bus_time provided was invalid");
  }
  try {
    test_csv_ = declare_parameter<bool>("test_csv", false);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The test_csv provided was invalid");
  }
  try {
    csv_path_ = declare_parameter<std::string>("csv_path", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The csv_path provided was invalid");
  }
  RCLCPP_INFO(get_logger(), "params: test_csv=%s, csv_path=%s", test_csv_ ? "true": "false", csv_path_.c_str());
}

void CanBridgeNode::receive()
{
  can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
  frame_msg.header.frame_id = "can";

  VCI_CAN_OBJ rec[3000];  // 接收缓存，设为3000为佳。
  driver_->open();
  driver_->initCan(can_idx_, static_cast<BTR>(can_baud_rate_));
  while (rclcpp::ok()) {
    if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    try {
      int reclen = driver_->receiveOnceToBuffer(can_idx_, rec, 3000);
      if (reclen > 0) {  // 调用接收函数，如果有数据，进行数据处理显示。
        RCLCPP_DEBUG(get_logger(), "receiveOnceToBuffer %d", reclen);
        for (int j = 0; j < reclen; j++) {
          if (use_bus_time_) {
            frame_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(
              rec[j].TimeStamp * 100000U));  // TimeStamp單位爲0.1毫秒，1毫秒=1000000納秒，
          } else {
            frame_msg.header.stamp = this->now();
          }
          frame_msg.id = rec[j].ID;
          frame_msg.is_rtr = rec[j].RemoteFlag == 1;
          frame_msg.is_extended = rec[j].ExternFlag == 1;
          frame_msg.is_error = false;
          frame_msg.dlc = rec[j].DataLen;
          std::copy(rec[j].Data, rec[j].Data + frame_msg.dlc, frame_msg.data.data());
          // std::move(data.begin(), data.end(), frame_msg.data.begin());
          frames_pub_->publish(std::move(frame_msg));
        }
      } else if (reclen < 0) {
        // 创芯CAN断线了。重新連接，
        RCLCPP_ERROR(get_logger(), "创芯CAN断线了。等待2秒, 重新連接");
        std::this_thread::sleep_for(2000ms);
        driver_->open();
        driver_->initCan(can_idx_, static_cast<BTR>(can_baud_rate_));
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Error receiving CAN message: %d - %s",
        can_idx_, ex.what());
      continue;
    }
  }
  driver_->close();
  RCLCPP_INFO(get_logger(), "创芯CAN接收線程退出");
}

void CanBridgeNode::on_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    try {
      RCLCPP_DEBUG(get_logger(), "on_frame %d: %d", can_idx_, msg->id);
      driver_->send(can_idx_, *msg);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Error sending CAN message: %d - %s",
        can_idx_, ex.what());
      return;
    }
  }
}

}  // namespace can_driver
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::can_driver::CanBridgeNode)
