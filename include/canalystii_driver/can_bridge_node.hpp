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

/**
 * 用於創芯Canalyst-II的對接
 * 1. 接收創芯CAN數據並上報
 * 1. 接收autoware發過來的CAN數據，下發到創芯CAN
 * 參數：
 * 1. device_index: Canalyst-II設備序號，從0開始
 * 1. can_index: CAN序號，從0開始
 */
#ifndef CAN_DRIVER__CAN_BRIDGE_NODE_HPP_
#define CAN_DRIVER__CAN_BRIDGE_NODE_HPP_

#include "canalystii_driver/canalystii.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace drivers
{
namespace can_driver
{

/// \brief CanBridgeNode class which can send and receive serial data
class CanBridgeNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  /// \param[in] options Options for the node
  explicit CanBridgeNode(const rclcpp::NodeOptions & options);

  // /// \brief Constructor which accepts IoContext
  // /// \param[in] options Options for the node
  // /// \param[in] ctx A shared IoContext
  // CanBridgeNode(
  //   const rclcpp::NodeOptions & options,
  //   const IoContext & ctx);

  /// \brief Destructor - required to manage owned IoContext
  ~CanBridgeNode();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for reading from hardware interface on timer tick.
  void receive();
  /// \brief Callback for ros can frame.
  void on_frame(const can_msgs::msg::Frame::SharedPtr msg);

private:
  void get_params();

  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> frames_pub_; // 接收創芯的消息，轉發到autoware
  std::unique_ptr<std::thread> receiver_thread_; // 接收創芯CAN消息的線程
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_; // 接收autoware發的CAN命令，轉發到CAN
  // std::unique_ptr<CANalystii> driver_;
  std::unique_ptr<BaseCAN> driver_;

  std::string frame_id_{"can"};
  uint32_t device_idx_{0};
  uint32_t can_idx_{0};
  uint32_t can_baud_rate_{0x1C00};
  bool use_bus_time_{false};
  bool test_csv_{false};
  std::string csv_path_{""};

};  // class CanBridgeNode

}  // namespace can_driver
}  // namespace drivers

#endif  // CAN_DRIVER__CAN_BRIDGE_NODE_HPP_
