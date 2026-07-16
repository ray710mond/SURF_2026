#include <algorithm>
#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

struct TimedPose
{
  rclcpp::Time stamp;
  tf2::Transform pose;
};

class MapOdomLocalizer : public rclcpp::Node
{
public:
  MapOdomLocalizer() : Node("map_odom_localizer")
  {
    robot_ = declare_parameter<std::string>("robot_name");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = robot_ + "/odom";
    base_z_offset_ = declare_parameter<double>("base_z_offset", 0.25);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_ + "/lio/odom", 100,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        TimedPose sample{rclcpp::Time(msg->header.stamp), tf2::Transform()};
        tf2::fromMsg(msg->pose.pose, sample.pose);
        odom_history_.push_back(sample);
        while (odom_history_.size() > 300) {odom_history_.pop_front();}
      });

    truth_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/" + robot_ + "/ground_truth_tf", rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (isRobotModel(transform.child_frame_id)) {
            publishCorrection(transform);
            return;
          }
        }
      });
  }

private:
  bool isRobotModel(const std::string & child) const
  {
    return child == robot_ || (child.size() >= robot_.size() &&
      child.compare(child.size() - robot_.size(), robot_.size(), robot_) == 0);
  }

  bool interpolatedOdom(const rclcpp::Time & stamp, tf2::Transform & result) const
  {
    if (odom_history_.empty()) {return false;}
    auto upper = std::lower_bound(
      odom_history_.begin(), odom_history_.end(), stamp,
      [](const TimedPose & sample, const rclcpp::Time & time) {
        return sample.stamp < time;
      });
    if (upper == odom_history_.begin()) {
      result = upper->pose;
      return std::abs((upper->stamp - stamp).seconds()) < 0.05;
    }
    if (upper == odom_history_.end()) {
      const auto & last = odom_history_.back();
      result = last.pose;
      return std::abs((last.stamp - stamp).seconds()) < 0.05;
    }

    const auto & after = *upper;
    const auto & before = *std::prev(upper);
    const double span = (after.stamp - before.stamp).seconds();
    if (span <= 0.0 || span > 0.1) {return false;}
    const double ratio = std::clamp(
      (stamp - before.stamp).seconds() / span, 0.0, 1.0);
    const tf2::Vector3 translation = before.pose.getOrigin().lerp(
      after.pose.getOrigin(), ratio);
    const tf2::Quaternion rotation = before.pose.getRotation().slerp(
      after.pose.getRotation(), ratio);
    result = tf2::Transform(rotation, translation);
    return true;
  }

  void publishCorrection(const geometry_msgs::msg::TransformStamped & truth)
  {
    const rclcpp::Time stamp(truth.header.stamp);
    tf2::Transform odom_to_base;
    if (!interpolatedOdom(stamp, odom_to_base)) {return;}

    tf2::Transform map_to_model;
    tf2::fromMsg(truth.transform, map_to_model);
    tf2::Transform model_to_base;
    model_to_base.setIdentity();
    model_to_base.setOrigin(tf2::Vector3(0.0, 0.0, base_z_offset_));

    const tf2::Transform map_to_odom =
      (map_to_model * model_to_base) * odom_to_base.inverse();
    geometry_msgs::msg::TransformStamped output;
    output.header.stamp = truth.header.stamp;
    output.header.frame_id = map_frame_;
    output.child_frame_id = odom_frame_;
    output.transform = tf2::toMsg(map_to_odom);
    broadcaster_->sendTransform(output);
  }

  std::string robot_, map_frame_, odom_frame_;
  double base_z_offset_;
  std::deque<TimedPose> odom_history_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr truth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomLocalizer>());
  rclcpp::shutdown();
  return 0;
}
