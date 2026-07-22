#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class SharedCloudMux : public rclcpp::Node
{
public:
  SharedCloudMux()
  : Node("shared_cloud_mux")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/shared/points");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    humanoid_subscription_ = subscribe("/humanoid/points");
    drone_subscription_ = subscribe("/drone/points");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscribe(
    const std::string & topic)
  {
    return create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
        // Preserve the source frame and timestamp. Bonxai uses both to transform
        // each scan and its sensor origin into the shared map frame.
        publisher_->publish(std::move(cloud));
      });
  }

  std::string output_topic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr humanoid_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr drone_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SharedCloudMux>());
  rclcpp::shutdown();
  return 0;
}
