#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring> // std::memcpy

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"
#include "interfaces/msg/vehicle_status.hpp"
#include "std_msgs/msg/int32.hpp"

//#include "nav_msgs/msg/odometry.hpp"
//#include "tf2_ros/transform_broadcaster.h"
//#include "tf2/LinearMath/Quaternion.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//#include "nav_msgs/msg/path.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"

const double DEG_TO_RAD = M_PI / 180.0;

// 차량 모드 정의
enum class VehicleMode {
    STANDBY,
    MANUAL,
    RECORDING,
    RETURNING,
    PARKING,
    EMERGENCY_STOP
};

// 디버깅용: enum을 문자열로 변환하는 헬퍼 함수
std::string to_string(VehicleMode mode) {
    switch (mode) {
        case VehicleMode::STANDBY: return "STANDBY";
        case VehicleMode::MANUAL: return "MANUAL";
        case VehicleMode::RECORDING: return "RECORDING";
        case VehicleMode::RETURNING: return "RETURNING";
        case VehicleMode::PARKING: return "PARKING";
        case VehicleMode::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

class CoreControllerNode : public rclcpp::Node
{
public:
    explicit CoreControllerNode(const rclcpp::NodeOptions & options) : Node("core_controller_node", options)
    {
	// 1. 파라미터 선언 (차량의 물리/성능 관련 파라미터만)
//        this->declare_parameter<double>("max_pwm", 255.0);
//        this->declare_parameter<double>("track_width", 0.13);
//        this->declare_parameter<double>("wheel_radius", 0.033);
        // 안전을 위한 최대 속도 제한값 (teleop_node와 자율 노드의 출력을 제한)
//        this->declare_parameter<double>("max_linear_velocity_limit", 0.88);
//        this->declare_parameter<double>("max_angular_velocity_limit", 13.5);

        // 2. 멤버 변수 초기화
        current_mode_ = VehicleMode::STANDBY;
//        last_odom_time_ = this->get_clock()->now();

        // 3. Publisher 및 Subscriber 선언
        can_tx_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
        vehicle_status_publisher_ = this->create_publisher<interfaces::msg::VehicleStatus>("/vehicle_status", 10);
//        odom_wheel_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/wheel", 10);
	left_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/left_wheel_rpm", 10);
        right_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/right_wheel_rpm", 10);

	manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10, std::bind(&CoreControllerNode::manual_cmd_callback, this, std::placeholders::_1));
        
        mode_request_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_request", 10, std::bind(&CoreControllerNode::mode_request_callback, this, std::placeholders::_1));

        aux_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/aux_command", 10, std::bind(&CoreControllerNode::aux_command_callback, this, std::placeholders::_1));

        can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/from_can_bus", 10, std::bind(&CoreControllerNode::can_rx_callback, this, std::placeholders::_1));

	auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_auto", 10, std::bind(&CoreControllerNode::auto_cmd_callback, this, std::placeholders::_1));

//	path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/recorded_path", 10);

	// 4. 기타 ROS 2 객체 초기화
//	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // TODO: 자율주행 노드가 발행할 토픽 구독
        // auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>( ... );

        // === 메인 제어 루프 타이머 ===
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&CoreControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Core Controller Node has been started in STANDBY mode.");
    }

private:
    // --- 멤버 변수 ---
    VehicleMode current_mode_;
    double target_v_ = 0.0; // 현재 목표 선속도 (m/s)
    double target_w_ = 0.0; // 현재 목표 각속도 (rad/s)
    double current_v_ = 0.0; // 피드백 받은 현재 선속도 (m/s)
    double current_w_ = 0.0; // 피드백 받은 현재 각속도 (rad/s)
    uint8_t current_aux_cmd_ = 0;

//    double odom_x_ = 0.0, odom_y_ = 0.0, odom_theta_ = 0.0;
//    rclcpp::Time last_odom_time_;

    // --- ROS 2 인터페이스 객체 ---
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_publisher_;
    rclcpp::Publisher<interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;
//    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_wheel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aux_command_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_sub_; 
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
//    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_rpm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_rpm_publisher_;


//    std::vector<geometry_msgs::msg::PoseStamped> recorded_path_;
//    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; // RViz2 시각화용

    // === 메인 제어 루프 ===
    void control_loop()
    {
        publish_vehicle_status(); // /vehicle_status 토픽 발행 (ROS 내부용)
        send_status_and_aux_to_can(); // TC375에 현재 상태 방송 (CAN용)

        switch (current_mode_)
        {
            case VehicleMode::MANUAL:
                handle_manual_mode();
                break;
            case VehicleMode::RECORDING:
                handle_recording_mode();
                break;
            case VehicleMode::PARKING:
                handle_parking_mode();
                break;
	        case VehicleMode::RETURNING:
                handle_autonomous_mode();
                break;
            case VehicleMode::EMERGENCY_STOP:
                handle_emergency_stop();
                break;
            default: // STANDBY
                handle_standby_mode();
                break;
        }
    }

    // === 모드별 처리 함수 ===
    void handle_standby_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In STANDBY mode...");
        send_pwm_command(0, 0);
    }

    void handle_manual_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In MANUAL mode...");
	convert_velocity_to_pwm_and_send(target_v_, target_w_);
    }

    void handle_parking_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In PARKING mode... waiting for TC375.");
        // 제어권은 TC375에 있으므로, RPi는 아무런 제어 명령도 보내지 않음.
    }

    void handle_recording_mode()
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RECORDING mode...");

	// 경로 기록은 odom_recorder_callback이 비동기적으로 알아서 처리하므로,
        // 이 함수에서는 특별히 경로 기록 로직을 호출할 필요가 없습니다.

        // 기록 중에도 수동 제어는 계속되어야 하므로, manual 핸들러를 호출
	convert_velocity_to_pwm_and_send(target_v_, target_w_);
    }

    void handle_autonomous_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In AUTONOMOUS mode...");
        // TODO: 자율주행 노드가 발행한 target_v_, target_w_를 사용
        convert_velocity_to_pwm_and_send(target_v_, target_w_);
    }

    void handle_emergency_stop() {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In EMERGENCY_STOP mode!");
        send_pwm_command(0, 0);
    }

    // === 콜백 함수 ===
    void manual_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (current_mode_ == VehicleMode::MANUAL || current_mode_ == VehicleMode::RECORDING) {
            target_v_ = msg->linear.x;
            target_w_ = msg->angular.z;
    	}
    }

    void mode_request_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string request = msg->data;
        RCLCPP_INFO(this->get_logger(), "Mode request received: '%s'", request.c_str());

        if (request == "toggle_manual") {
            if (current_mode_ == VehicleMode::STANDBY) { switch_mode(VehicleMode::MANUAL); }
            else if (current_mode_ == VehicleMode::MANUAL) { switch_mode(VehicleMode::STANDBY); }
        }
        else if (request == "request_parking") {
            if (current_mode_ == VehicleMode::MANUAL && std::abs(this->current_v_) < 0.05) {
                RCLCPP_INFO(this->get_logger(), "Switching to PARKING mode. TC375 will take over.");
                switch_mode(VehicleMode::PARKING);
            } else {
                 RCLCPP_WARN(this->get_logger(), "Cannot start parking. Vehicle must be in MANUAL mode and nearly stopped.");
            }
        } else if (request == "request_path_record_or_return") {
            if (current_mode_ == VehicleMode::MANUAL) { switch_mode(VehicleMode::RECORDING); } 
            else if (current_mode_ == VehicleMode::RECORDING) { switch_mode(VehicleMode::RETURNING); }
        } else if (request == "request_cancel") {
            // PARKING 취소 시에는 별도의 CAN 명령 없이 상태만 변경
            if (current_mode_ != VehicleMode::STANDBY && current_mode_ != VehicleMode::MANUAL) {
                switch_mode(VehicleMode::MANUAL);
            }
        }
    }

    void aux_command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        this->current_aux_cmd_ = msg->data;
    }

    void can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if (msg->id == 0x201 && msg->dlc == 8) {
            int32_t left_rpm_val, right_rpm_val;
            std::memcpy(&left_rpm_val, &msg->data[0], sizeof(int32_t));
            std::memcpy(&right_rpm_val, &msg->data[4], sizeof(int32_t));

            auto left_rpm_msg = std_msgs::msg::Int32();
            left_rpm_msg.data = left_rpm_val;
            left_rpm_publisher_->publish(left_rpm_msg);

            auto right_rpm_msg = std_msgs::msg::Int32();
            right_rpm_msg.data = right_rpm_val;
            right_rpm_publisher_->publish(right_rpm_msg);
        }
        // TODO: ID 0x300 (긴급 제동) 등 다른 CAN ID 처리
    }

    void auto_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 자율주행 모드일 때만 목표 속도를 업데이트
        if (current_mode_ == VehicleMode::RETURNING || current_mode_ == VehicleMode::PARKING) {
            this->target_v_ = msg->linear.x;
            this->target_w_ = msg->angular.z;
        }
    }

    // --- 유틸리티 함수 ---
    void convert_velocity_to_pwm_and_send(double v, double w)
    {
        // 파라미터 가져오기
        const double MAX_PWM = this->get_parameter("max_pwm").as_double();
        const double TRACK_WIDTH = this->get_parameter("track_width").as_double();
        const double WHEEL_RADIUS = this->get_parameter("wheel_radius").as_double();
        const double MAX_LINEAR_VEL_LIMIT = this->get_parameter("max_linear_velocity_limit").as_double();
        // YAML에서 deg/s 단위의 파라미터를 읽어옴
        const double MAX_ANGULAR_VEL_LIMIT_DEGS = this->get_parameter("max_angular_velocity_limit").as_double();
        // 즉시 rad/s 단위로 변환하여 사용
        const double MAX_ANGULAR_VEL_LIMIT_RADS = MAX_ANGULAR_VEL_LIMIT_DEGS * DEG_TO_RAD;

	// 안전을 위해 들어온 목표 속도를 제한 (이제 단위가 일치함)
        v = std::max(-MAX_LINEAR_VEL_LIMIT, std::min(v, MAX_LINEAR_VEL_LIMIT));
        w = std::max(-MAX_ANGULAR_VEL_LIMIT_RADS, std::min(w, MAX_ANGULAR_VEL_LIMIT_RADS));

        // Inverse Kinematics: (v, w) -> (v_L, v_R)
        double v_left = v - (w * TRACK_WIDTH / 2.0);
        double v_right = v + (w * TRACK_WIDTH / 2.0);

        // 바퀴 선속도(m/s) -> 바퀴 각속도(rad/s) 변환
        double w_left = v_left / WHEEL_RADIUS;
        double w_right = v_right / WHEEL_RADIUS;

        // 모터가 낼 수 있는 최대 각속도 계산
        double max_wheel_angular_vel = MAX_LINEAR_VEL_LIMIT / WHEEL_RADIUS;

        // 바퀴 각속도를 -1.0 ~ 1.0 비율로 정규화
        double left_ratio = (max_wheel_angular_vel > 1e-6) ? (w_left / max_wheel_angular_vel) : 0.0;
        double right_ratio = (max_wheel_angular_vel > 1e-6) ? (w_right / max_wheel_angular_vel) : 0.0;

        // 출력 정규화
        double max_abs_ratio = std::max(std::abs(left_ratio), std::abs(right_ratio));
        if (max_abs_ratio > 1.0) {
            left_ratio /= max_abs_ratio;
            right_ratio /= max_abs_ratio;
        }

        // PWM 값으로 최종 변환
        int left_pwm = static_cast<int>(left_ratio * MAX_PWM);
        int right_pwm = static_cast<int>(right_ratio * MAX_PWM);

        send_pwm_command(left_pwm, right_pwm);
    }

    void send_pwm_command(int left_pwm, int right_pwm)
    {
        auto frame = can_msgs::msg::Frame();
        frame.id = 0x100;
        frame.dlc = 4;

        frame.data[0] = (left_pwm >= 0) ? 1 : 0;
        frame.data[1] = static_cast<uint8_t>(std::abs(left_pwm));
        frame.data[2] = (right_pwm >= 0) ? 1 : 0;
        frame.data[3] = static_cast<uint8_t>(std::abs(right_pwm));

        can_tx_publisher_->publish(frame);
    }

    void send_status_and_aux_to_can()
    {
        auto frame = can_msgs::msg::Frame();
        frame.id = 0x102;
        frame.dlc = 3;

        frame.data[0] = static_cast<uint8_t>(current_mode_);
        frame.data[1] = current_aux_cmd_;
        frame.data[2] = 0; // 부저 (향후 확장)

        can_tx_publisher_->publish(frame);
    }

    void publish_vehicle_status()
    {
        auto status_msg = interfaces::msg::VehicleStatus();
        status_msg.mode = to_string(current_mode_);

	// can_rx_callback -> calculate_and_publish_odometry를 통해
        // 이미 계산 및 업데이트된 최신 속도 값을 사용
        status_msg.current_velocity.linear.x = this->current_v_;
        status_msg.current_velocity.angular.z = this->current_w_;

        vehicle_status_publisher_->publish(status_msg);
    }

    void switch_mode(VehicleMode new_mode) {
        if (current_mode_ != new_mode) {
            RCLCPP_INFO(this->get_logger(), "Switching mode from %s to %s",
                        to_string(current_mode_).c_str(), to_string(new_mode).c_str());
            current_mode_ = new_mode;

            // 모드가 바뀌면 항상 제어 명령을 0으로 리셋 (안전 장치)
            target_v_ = 0.0;
            target_w_ = 0.0;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<CoreControllerNode>(options));
///    rclcpp::shutdown();
    return 0;
}
