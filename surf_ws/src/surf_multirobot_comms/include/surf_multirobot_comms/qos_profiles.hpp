#pragma once

#include "rclcpp/qos.hpp"

namespace surf::comms
{

inline rclcpp::QoS realtime_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
}

inline rclcpp::QoS sync_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
}

}  // namespace surf::comms
