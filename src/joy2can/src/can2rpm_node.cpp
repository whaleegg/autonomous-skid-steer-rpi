#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "can_msgs/msg/frame.hpp"

class Can2RpmNode : public rclcpp::Node {
public:
	Can2RpmNode() : Node("can_to_rpm_node") {
		left_pub_ = this->create_publisher<std_msgs::msg::Int32>("/left_wheel_rpm", 10);
		right_pub_ = this->create_publisher<std_msgs::msg::Int32>("/right_wheel_rpm", 10);
		sub_ = this->create_subscription<can_msgs::msg::Frame>(
				"/from_can_bus", 10,
				std::bind(&Can2RpmNode::can_callback, this, std::placeholders::_1)
				);
	}
private:
	void can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
		if (msg->id == 0x201) {
			std_msgs::msg::Int32 left_msg;
			std_msgs::msg::Int32 right_msg;

			left_msg.data =
				static_cast<int32_t>(msg->data[0])
				| (static_cast<int32_t>(msg->data[1]) << 8)
				| (static_cast<int32_t>(msg->data[2]) << 16)
				| (static_cast<int32_t>(msg->data[3]) << 24);

			right_msg.data =
				static_cast<int32_t>(msg->data[4])
				| (static_cast<int32_t>(msg->data[5]) << 8)
				| (static_cast<int32_t>(msg->data[6]) << 16)
				| (static_cast<int32_t>(msg->data[7]) << 24);

			left_pub_->publish(left_msg);
			right_pub_->publish(right_msg);
		}
	}

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_pub_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_pub_;
	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Can2RpmNode>());
	rclcpp::shutdown();
	return 0;
}
