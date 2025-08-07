#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "can_msgs/msg/frame.hpp"

class Joy2CanNode : public rclcpp::Node {
public:
	Joy2CanNode() : Node("joy_to_can_node"), buttons_prev({0, }), stopped(1), dirA(1), dirB(1) {
		pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
		sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
				"/joy", 10,
				std::bind(&Joy2CanNode::joy_callback, this, std::placeholders::_1)
				);
	}
private:
	uint8_t buttons_prev[8];
	int stopped;
	uint8_t dirA, dirB;
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		if (msg->axes.size() < 4)
			return;

		can_msgs::msg::Frame can_msg;
		can_msg.is_rtr = false;
		can_msg.is_extended = false;
		can_msg.is_error = false;
		can_msg.header.frame_id = "base_link";

		int send = 0;
		for (int i = 0; i < 8; i++)
			can_msg.data[i] = 0;

		if (buttons_prev[0] == 0 && msg->buttons[0] == 1) {
			can_msg.data[0] = 1;
			send++;
		}
		buttons_prev[0] = msg->buttons[0];

		if (buttons_prev[1] == 0 && msg->buttons[1] == 1) {
			can_msg.data[1] = 1;
			send++;
		}
		buttons_prev[1] = msg->buttons[1];

		if (buttons_prev[2] == 0 && msg->buttons[3] == 1) {
			can_msg.data[2] = 1;
			send++;
		}
		buttons_prev[2] = msg->buttons[3];

		if (buttons_prev[3] == 0 && msg->buttons[2] == 1) {
			can_msg.data[3] = 1;
			send++;
		}
		buttons_prev[3] = msg->buttons[2];

		if (buttons_prev[4] == 0 && msg->buttons[4] == 1) {
			can_msg.data[4] = 1;
			send++;
		}
		buttons_prev[4] = msg->buttons[4];

		if (buttons_prev[5] == 0 && msg->buttons[5] == 1) {
			can_msg.data[5] = 1;
			send++;
		}
		buttons_prev[5] = msg->buttons[5];

		if (buttons_prev[6] == 0 && msg->buttons[12] == 1) {
			can_msg.data[6] = 1;
			send++;
		}
		buttons_prev[6] = msg->buttons[12];

		if (send) {
			can_msg.id = 0x102;
			can_msg.dlc = 7;
			can_msg.header.stamp = this->now();
			pub_->publish(can_msg);
			RCLCPP_INFO(this->get_logger(), "buttons message published");
		}

		if (std::fabs(msg->axes[1]) < 1e-4f) {
			if (stopped)
				return;
			else
				stopped = 1;
		} else
			stopped = 0;

		int OFFSET = 64;
		float drive = (255-OFFSET) * msg->axes[1];
		if (drive > 0) {
			drive += OFFSET;
		} else if (drive < 0) {
			drive -= OFFSET;
		}
		float diff = drive * msg->axes[3];
		if (drive < 0)
			diff *= -1;
		int pwmA = static_cast<int>(std::round(drive - diff));
		int pwmB = static_cast<int>(std::round(drive + diff));

		if (pwmA > 255)
			pwmA = 255;
		else if (pwmA < -255)
			pwmA = -255;
		if (pwmB > 255)
			pwmB = 255;
		else if (pwmB < -255)
			pwmB = -255;

		if (pwmA < 0) {
			can_msg.data[0] = 0;
			can_msg.data[1] = static_cast<uint8_t>(-pwmA);
			dirA = 0;
		} else if (pwmA > 0) {
			can_msg.data[0] = 1;
			can_msg.data[1] = static_cast<uint8_t>(pwmA);
			dirA = 1;
		} else {
			can_msg.data[0] = dirA;
			can_msg.data[1] = 0;
		}

		if (pwmB < 0) {
			can_msg.data[2] = 0;
			can_msg.data[3] = static_cast<uint8_t>(-pwmB);
			dirB = 0;
		} else if (pwmB > 0) {
			can_msg.data[2] = 1;
			can_msg.data[3] = static_cast<uint8_t>(pwmB);
			dirB = 1;
		} else {
			can_msg.data[2] = dirB;
			can_msg.data[3] = 0;
		}

		for (int i = 4; i < 8; i++)
			can_msg.data[i] = 0;

		can_msg.id = 0x100;
		can_msg.dlc = 4;
		can_msg.header.stamp = this->now();

		pub_->publish(can_msg);
	}

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Joy2CanNode>());
	rclcpp::shutdown();
	return 0;
}
