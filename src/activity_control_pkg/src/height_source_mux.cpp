#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include <memory>
#include <string>

namespace activity_control_pkg {

class HeightSourceMux final : public rclcpp::Node {
 public:
  HeightSourceMux() : Node("height_source_mux") {
    const auto height_source = declare_parameter<std::string>(
      "height_source", "laser_array");
    const auto laser_height_topic = declare_parameter<std::string>(
      "laser_height_topic", "/height_laser_array");
    const auto stm32_height_topic = declare_parameter<std::string>(
      "stm32_height_topic", "/height_stm32");
    const auto output_height_topic = declare_parameter<std::string>(
      "output_height_topic", "/height");

    height_pub_ = create_publisher<std_msgs::msg::Int16>(output_height_topic, rclcpp::QoS(10));

    std::string selected_topic;
    if (height_source == "laser_array") {
      selected_topic = laser_height_topic;
    } else if (height_source == "stm32") {
      selected_topic = stm32_height_topic;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid height_source '%s'; use 'laser_array' or 'stm32'. No /height data will be published.",
        height_source.c_str());
      return;
    }

    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      selected_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::Int16::SharedPtr message) {
        height_pub_->publish(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Height source selected: %s (%s -> %s), unit: cm.",
      height_source.c_str(), selected_topic.c_str(), output_height_topic.c_str());
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
};

}  // namespace activity_control_pkg

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::HeightSourceMux>());
  rclcpp::shutdown();
  return 0;
}
