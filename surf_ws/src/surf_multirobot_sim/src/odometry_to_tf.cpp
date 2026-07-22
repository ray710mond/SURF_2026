#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryToTf : public rclcpp::Node
{
public:
  OdometryToTf()
  : Node("odometry_to_tf")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::vector<std::string> robot_names = {"humanoid", "drone"};
    for (const auto &robot_name : robot_names) {
      const std::string odom_topic = "/" + robot_name + "/lio/odom";
      subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        rclcpp::QoS(10),
        [this, robot_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
          publish_transform(robot_name, msg);
        }));
    }
  }

private:
  void publish_transform(
    const std::string &robot_name,
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = robot_name + "/odom";
    tf_msg.child_frame_id = robot_name + "/base_link";

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToTf>());
  rclcpp::shutdown();
  return 0;
}
