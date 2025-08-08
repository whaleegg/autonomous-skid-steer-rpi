#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "yaml-cpp/yaml.h"

// 캘리브레이션 데이터를 저장하기 위한 구조체
struct AxisCalibration {
    double center = 0.0;
    double deadzone_min = 0.0;
    double deadzone_max = 0.0;
    double min = -1.0;
    double max = 1.0;
};

// 수동 조작 방식을 위한 enum class
enum class SteeringMode {
    SKID_STEER,
    ACKERMANN_SIM
};

class TeleopNode : public rclcpp::Node
{
public:
//    TeleopNode() : Node("teleop_node")
    explicit TeleopNode(const rclcpp::NodeOptions & options) : Node("teleop_node", options)
    {
	// === 파라미터 선언 ===
//        this->declare_parameter<std::string>("steering_mode", "ackermann_sim");
//        this->declare_parameter<std::string>("calibration_file", "/root/ros_ws/src/joystick_cal.yaml");
//        this->declare_parameter<double>("expo_factor", 0.7);
        
//        this->declare_parameter<int>("axis_linear", 1);
//        this->declare_parameter<int>("axis_angular", 2);
//        this->declare_parameter<double>("scale_linear", 1.0);
//        this->declare_parameter<double>("scale_angular", 1.0);
        
//        this->declare_parameter<int>("button_toggle_manual", 1);
//        this->declare_parameter<int>("button_request_parking", 2);
//        this->declare_parameter<int>("button_request_path", 3);
//        this->declare_parameter<int>("button_request_cancel", 0);
//        this->declare_parameter<int>("button_left_blinker", 4);
//        this->declare_parameter<int>("button_right_blinker", 5);
//        this->declare_parameter<int>("button_toggle_steering_1", 10);
//        this->declare_parameter<int>("button_toggle_steering_2", 11);

//        this->declare_parameter<double>("skid_max_linear_velocity", 0.88);
//        this->declare_parameter<double>("skid_max_angular_velocity", 13.54);
//        this->declare_parameter<double>("linear_sensitivity", 1.0);
//        this->declare_parameter<double>("steering_sensitivity", 0.7);

//        this->declare_parameter<double>("wheelbase", 0.13);
//        this->declare_parameter<double>("max_steering_angle", 35.0); // degree 단위로 받음
//        this->declare_parameter<double>("ackermann_max_linear_velocity", 0.88);

	// === 2. 캘리브레이션 파일 로드 ===
        load_calibration_file();

	// === 현재 조작 모드 설정 ===
        std::string mode_str = this->get_parameter("steering_mode").as_string();
        if (mode_str == "skid_steer") {
            current_steering_mode_ = SteeringMode::SKID_STEER;
        } else {
            current_steering_mode_ = SteeringMode::ACKERMANN_SIM;
        }
        RCLCPP_INFO(this->get_logger(), "Starting in %s steering mode.", mode_str.c_str());

        // === Publisher/Subscriber 선언 ===
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
        mode_request_pub_ = this->create_publisher<std_msgs::msg::String>("/mode_request", 10);
        aux_command_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/aux_command", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));
//	steering_mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/steering_mode", 10);

        last_buttons_.resize(12, 0);
        RCLCPP_INFO(this->get_logger(), "Teleop Node has been started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
	handle_buttons(msg);
        
        size_t axis_linear_idx = this->get_parameter("axis_linear").as_int();
        size_t axis_angular_idx = this->get_parameter("axis_angular").as_int();
        if (msg->axes.size() <= axis_linear_idx || msg->axes.size() <= axis_angular_idx) return;

        double normalized_linear = normalize_axis(msg->axes[axis_linear_idx], axis_linear_idx);
        double normalized_angular = normalize_axis(msg->axes[axis_angular_idx], axis_angular_idx);
        
        auto twist_msg = geometry_msgs::msg::Twist();

        if (current_steering_mode_ == SteeringMode::ACKERMANN_SIM) {
            calculate_ackermann_twist(normalized_linear, normalized_angular, twist_msg);
        } else { // SKID_STEER
            calculate_skid_steer_twist(normalized_linear, normalized_angular, twist_msg);
        }
        
        cmd_vel_pub_->publish(twist_msg);
        
//	auto mode_msg = std_msgs::msg::String();
//	mode_msg.data = (current_steering_mode_ == SteeringMode::SKID_STEER) ? "skid_steer" : "ackermann_sim";
//	steering_mode_publisher_->publish(mode_msg);

        last_buttons_ = msg->buttons;
    }

    void calculate_skid_steer_twist(double n_linear, double n_angular, geometry_msgs::msg::Twist& twist_out)
    {
        double linear_input = n_linear * this->get_parameter("linear_sensitivity").as_double();
        double angular_input = n_angular * this->get_parameter("steering_sensitivity").as_double();
        double magnitude = std::sqrt(linear_input * linear_input + angular_input * angular_input);
        if (magnitude > 1.0) {
            linear_input /= magnitude;
            angular_input /= magnitude;
        }
        double curved_linear = apply_response_curve(linear_input);
        double curved_angular = apply_response_curve(angular_input);
        const double MAX_LINEAR_VEL = this->get_parameter("skid_max_linear_velocity").as_double();
        const double MAX_ANGULAR_VEL = this->get_parameter("skid_max_angular_velocity").as_double();
        twist_out.linear.x = this->get_parameter("scale_linear").as_double() * curved_linear * MAX_LINEAR_VEL;
        twist_out.angular.z = this->get_parameter("scale_angular").as_double() * curved_angular * MAX_ANGULAR_VEL;
    }

    void calculate_ackermann_twist(double n_linear, double n_angular, geometry_msgs::msg::Twist& twist_out)
    {
        double throttle_input = apply_response_curve(n_linear);
        double steering_input = apply_response_curve(n_angular);

        const double MAX_LINEAR_VEL = this->get_parameter("ackermann_max_linear_velocity").as_double();
        const double WHEELBASE = this->get_parameter("wheelbase").as_double();
        const double MAX_STEERING_ANGLE_DEG = this->get_parameter("max_steering_angle").as_double();
        const double MAX_STEERING_ANGLE_RAD = MAX_STEERING_ANGLE_DEG * (M_PI / 180.0);
        
        double steering_angle = steering_input * MAX_STEERING_ANGLE_RAD;
        
        twist_out.linear.x = this->get_parameter("scale_linear").as_double() * throttle_input * MAX_LINEAR_VEL;
        twist_out.angular.z = (std::abs(WHEELBASE) > 1e-6) ? (twist_out.linear.x * std::tan(steering_angle) / WHEELBASE) : 0.0;
        twist_out.angular.z *= this->get_parameter("scale_angular").as_double();
    }

    void handle_buttons(const sensor_msgs::msg::Joy::SharedPtr& msg)
    {
        if (msg->buttons.size() < 12) return;

        // === 1. 조작 모드(skid/ackermann) 전환 처리 ===
        size_t btn_l3_idx = this->get_parameter("button_toggle_steering_1").as_int();
        size_t btn_r3_idx = this->get_parameter("button_toggle_steering_2").as_int();
        bool both_pressed_now = (msg->buttons[btn_l3_idx] == 1 && msg->buttons[btn_r3_idx] == 1);
        // last_buttons_ 벡터 크기가 충분한지 확인하여 에러 방지
        bool both_pressed_before = (last_buttons_.size() > btn_r3_idx) && 
                                   (last_buttons_[btn_l3_idx] == 1 && last_buttons_[btn_r3_idx] == 1);

        if (both_pressed_now && !both_pressed_before) {
            if (current_steering_mode_ == SteeringMode::ACKERMANN_SIM) {
                current_steering_mode_ = SteeringMode::SKID_STEER;
                RCLCPP_INFO(this->get_logger(), "--> Switched to SKID_STEER steering mode <--");
            } else {
                current_steering_mode_ = SteeringMode::ACKERMANN_SIM;
                RCLCPP_INFO(this->get_logger(), "--> Switched to ACKERMANN_SIM steering mode <--");
            }
        }

        // === 2. 시스템 상태(MANUAL/STANDBY 등) 변경 요청 처리 ===
        if (is_button_pressed(msg, this->get_parameter("button_toggle_manual").as_int())) {
            publish_mode_request("toggle_manual");
        }
        if (is_button_pressed(msg, this->get_parameter("button_request_parking").as_int())) {
            publish_mode_request("request_parking");
        }
        if (is_button_pressed(msg, this->get_parameter("button_request_path").as_int())) {
            publish_mode_request("request_path_record_or_return");
        }
        if (is_button_pressed(msg, this->get_parameter("button_request_cancel").as_int())) {
            publish_mode_request("request_cancel");
        }
        
        // === 3. 보조 기능(깜빡이) 처리 ===
        uint8_t aux_cmd = 0;
        if (msg->buttons[this->get_parameter("button_left_blinker").as_int()] == 1) {
            aux_cmd |= (1 << 0); // bit 0: left blinker
        }
        if (msg->buttons[this->get_parameter("button_right_blinker").as_int()] == 1) {
            aux_cmd |= (1 << 1); // bit 1: right blinker
        }

        if (aux_cmd != last_aux_cmd_) {
            auto aux_msg = std_msgs::msg::UInt8();
            aux_msg.data = aux_cmd;
            aux_command_pub_->publish(aux_msg);
            last_aux_cmd_ = aux_cmd;
        }
    }

    double normalize_axis(double raw_value, size_t axis_index)
    {
        if (calibration_data_.find(axis_index) == calibration_data_.end()) {
            return raw_value;
        }

        const auto& cal = calibration_data_.at(axis_index);

        if (raw_value > cal.deadzone_min && raw_value < cal.deadzone_max) {
            return 0.0;
        }

        double normalized_value = 0.0;
        if (raw_value >= cal.deadzone_max) {
            double effective_range = cal.max - cal.deadzone_max;
            if (effective_range > 1e-6)
                normalized_value = (raw_value - cal.deadzone_max) / effective_range;
            else
                normalized_value = 1.0;
        } else {
            double effective_range = cal.deadzone_min - cal.min;
            if (effective_range > 1e-6)
                normalized_value = (raw_value - cal.deadzone_min) / effective_range;
            else
                normalized_value = -1.0;
        }
        normalized_value = std::max(-1.0, std::min(normalized_value, 1.0));

        return apply_response_curve(normalized_value);
    }
    
    double apply_response_curve(double value)
    {
        double expo_factor = this->get_parameter("expo_factor").as_double();
        return expo_factor * std::pow(value, 3) + (1.0 - expo_factor) * value;
    }

    void load_calibration_file()
    {
        std::string file_path = this->get_parameter("calibration_file").as_string();
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            const auto& axes = config["axes"];
            for (const auto& axis_node : axes) {
                int axis_index = axis_node.first.as<int>();
                const auto& cal = axis_node.second;
                calibration_data_[axis_index] = {
                    cal["center"].as<double>(),
                    cal["deadzone_min"].as<double>(),
                    cal["deadzone_max"].as<double>(),
                    cal["min"].as<double>(),
                    cal["max"].as<double>()
                };
            }
            RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration file from %s", file_path.c_str());
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or parse calibration file %s: %s. Using raw values.", file_path.c_str(), e.what());
        }
    }
    
    // ��ư�� '��� ���ȴ���' Ȯ���ϴ� ���� �Լ�
    bool is_button_pressed(const sensor_msgs::msg::Joy::SharedPtr& msg, int index)
    {
        return msg->buttons[index] == 1 && last_buttons_[index] == 0;
    }
    
    // ��� ��û �޽����� �����ϴ� ���� �Լ�
    void publish_mode_request(const std::string& request)
    {
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = request;
        mode_request_pub_->publish(mode_msg);
        RCLCPP_INFO(this->get_logger(), "Published mode request: %s", request.c_str());
    }

    // --- 멤버 변수 ---
    SteeringMode current_steering_mode_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_request_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aux_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    std::vector<int> last_buttons_;
    std::map<int, AxisCalibration> calibration_data_; // Ķ���극�̼� ������ ����� map

    uint8_t last_aux_cmd_ = 0; // 이전 aux_command 상태 저장
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // NodeOptions를 생성하고, 파라미터를 자동으로 선언하도록 설정
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    // 수정된 생성자에 options 전달
    rclcpp::spin(std::make_shared<TeleopNode>(options));
    rclcpp::shutdown();
    return 0;
}
