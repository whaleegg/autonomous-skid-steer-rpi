#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"
#include "interfaces/msg/vehicle_status.hpp"
#include "std_msgs/msg/int64.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cstring> // std::memcpy

// 1. C++의 enum class를 사용하여 차량 모드를 안전하게 정의
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
    //CoreControllerNode() : Node("core_controller_node")
    explicit CoreControllerNode(const rclcpp::NodeOptions & options) : Node("core_controller_node", options)
    {
//	// 1. 파라미터 선언 (차량의 물리/성능 관련 파라미터만)
//        this->declare_parameter<double>("max_pwm", 255.0);
//	// 자율제어
//        this->declare_parameter<double>("track_width", 0.13);
//        this->declare_parameter<double>("max_linear_velocity", 0.8);
//        this->declare_parameter<double>("max_angular_velocity", 2.0);

        // 2. 현재 모드를 STANDBY로 초기화
        current_mode_ = VehicleMode::STANDBY;

        // 3. Publisher 및 Subscriber 선언
        can_tx_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
        vehicle_status_publisher_ = this->create_publisher<interfaces::msg::VehicleStatus>("/vehicle_status", 10);

    	// --- 여기에 RPM Publisher 생성 코드를 추가합니다 ---
        left_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/left_wheel_rpm", 10);
        right_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/right_wheel_rpm", 10);

    	odom_wheel_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/wheel", 10);
	    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    	last_odom_time_ = this->get_clock()->now();



        manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10, std::bind(&CoreControllerNode::manual_cmd_callback, this, std::placeholders::_1));
        
        mode_request_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_request", 10, std::bind(&CoreControllerNode::mode_request_callback, this, std::placeholders::_1));

        aux_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/aux_command", 10, std::bind(&CoreControllerNode::aux_command_callback, this, std::placeholders::_1));

        // TODO: 자율주행 노드가 발행할 토픽 구독
        // auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>( ... );
        
	// TC375의 피드백을 받을 토픽 구독
        can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/from_can_bus", 10, std::bind(&CoreControllerNode::can_rx_callback, this, std::placeholders::_1));

        // === 메인 제어 루프 타이머 ===
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&CoreControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Core Controller Node has been started in STANDBY mode.");
    }

private:
    // ==================== 멤버 변수 (상태 저장용) ====================
    VehicleMode current_mode_;

    // --- 수동 제어용 ---
    double latest_linear_vel_ratio_ = 0.0;
    double latest_angular_vel_ratio_ = 0.0;

    // --- 자율 제어용 (TODO) ---
    // double latest_auto_linear_vel_ = 0.0;
    // double latest_auto_angular_vel_ = 0.0;

    // --- 오도메트리 계산용 ---
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_theta_ = 0.0;
    rclcpp::Time last_odom_time_;

    // ==================== ROS 2 인터페이스 객체 ====================
    // --- Publishers ---
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_publisher_;
    rclcpp::Publisher<interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_rpm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_rpm_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_wheel_publisher_;
    
    // --- Subscribers ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aux_command_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;

    // --- 기타 ---
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    // ==================== 함수 선언 ====================
    // === 메인 제어 루프 ===
    void control_loop()
    {
        publish_vehicle_status();

        // 4. 상태 머신의 핵심: 현재 모드에 따라 적절한 함수 호출
        switch (current_mode_)
        {
            case VehicleMode::MANUAL:
		handle_manual_mode();
                break;
            case VehicleMode::PARKING:
                // handle_parking_mode();
		break;
	    case VehicleMode::RECORDING: // 기록 중에도 수동 제어는 계속되어야 함
                handle_recording_mode();
                break;
	    case VehicleMode::RETURNING:
                handle_returning_mode();
                break;
            case VehicleMode::EMERGENCY_STOP:
                handle_emergency_stop();
                break;
            // ... 다른 모드들 ...
            default: // STANDBY 및 기타 모든 경우
                handle_standby_mode();
                break;
        }
    }

    // === 모드별 처리 함수 (뼈대) ===
    void handle_standby_mode() {
        //RCLCPP_INFO(this->get_logger(), "In STANDBY mode...", RCLC_LOG_THROTTLE(RCUTILS_STEADY_TIME, 1000));
	RCLCPP_INFO_THROTTLE(
            this->get_logger(), // [1] 로거
            *this->get_clock(), // [2] 시간 소스 (시계)
            1000,               // [3] 시간 간격 (milliseconds)
            "In STANDBY mode..." // [4] 로그 메시지
    	);
	send_pwm_command(0, 0);
    }

    void handle_manual_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In MANUAL mode...");
	handle_manual_mode_logic();
    }

    void handle_parking_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In PARKING mode...");
        // TODO: /cmd_vel_auto 토픽의 값을 PWM으로 변환하여 전송
	// handle_autonomous_mode_logic();
	// 지금은 tc375에서 수행
    }

    void handle_recording_mode()
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RECORDING mode...");
        
	// 2. TODO: 여기에 path_recorder_node와 상호작용하여
        //           경로를 기록하라는 신호를 보내는 로직이 들어갈 것
        //    (예: path_recorder->add_current_pose();)

        // 기록 중에도 수동 제어는 계속되어야 하므로, manual 핸들러를 호출
        handle_manual_mode_logic(); // 기존 manual 핸들러의 내용을 별도 함수로 분리
    }

    void handle_returning_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RETURNING mode...");
        // TODO: /cmd_vel_auto 토픽의 값을 PWM으로 변환하여 전송
	handle_autonomous_mode_logic();
    }

    void handle_emergency_stop() {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In EMERGENCY_STOP mode!");
        // 모든 제어 명령 전송을 중단하거나, 명시적인 정지 명령을 보냄
        send_pwm_command(0, 0);
    }

    void handle_manual_mode_logic()
    {
	// 1. 파라미터 가져오기
        const double MAX_PWM = this->get_parameter("max_pwm").as_double();

        // 2. teleop_node가 보낸 최종 보정된 비율 값을 가져옴
        double linear_final_ratio = this->latest_linear_vel_ratio_;
        double angular_final_ratio = this->latest_angular_vel_ratio_;
        
        // 3. 스키드 스티어 믹싱
        double left_signal = linear_final_ratio - angular_final_ratio;
        double right_signal = linear_final_ratio + angular_final_ratio;

        // 4. 최종 클리핑
        // (teleop_node에서 이미 정규화되었지만, 안전을 위해 한번 더 수행)
        left_signal = std::max(-1.0, std::min(left_signal, 1.0));
        right_signal = std::max(-1.0, std::min(right_signal, 1.0));
        
        // 5. PWM 값으로 최종 변환
        int left_pwm = static_cast<int>(left_signal * MAX_PWM);
        int right_pwm = static_cast<int>(right_signal * MAX_PWM);
        
        // 6. CAN 메시지 전송
        send_pwm_command(left_pwm, right_pwm);
    }

    void handle_autonomous_mode_logic()
    {
        // TODO: 자율 제어를 위한 물리 모델 기반 PWM 변환 로직
    }

    // === 콜백 함수 ===
    void manual_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_linear_vel_ratio_ = msg->linear.x;
        latest_angular_vel_ratio_ = msg->angular.z;
    }

    void mode_request_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string request = msg->data;
        RCLCPP_INFO(this->get_logger(), "Mode request received: '%s'", request.c_str());

        if (request == "toggle_manual") {
            if (current_mode_ == VehicleMode::STANDBY) {
                switch_mode(VehicleMode::MANUAL);
            } else if (current_mode_ == VehicleMode::MANUAL) {
                switch_mode(VehicleMode::STANDBY);
            }
        }
        // ㅁ 버튼: 자율 주차 시작
        else if (request == "request_parking") {
            if (current_mode_ == VehicleMode::MANUAL) {
                // TODO: 자율 주차 시작 조건 확인 (예: 저속, 직선 주행 중)
                switch_mode(VehicleMode::PARKING);
            }
        }
        // △ 버튼: 경로 기록 또는 복귀
        else if (request == "request_path_record_or_return") {
            if (current_mode_ == VehicleMode::MANUAL) {
                switch_mode(VehicleMode::RECORDING);
            } else if (current_mode_ == VehicleMode::RECORDING) {
                switch_mode(VehicleMode::RETURNING);
            } else if (current_mode_ == VehicleMode::RETURNING) {
                switch_mode(VehicleMode::MANUAL);
            }
        }
        // X 버튼: 모든 작업 취소
        else if (request == "request_cancel") {
            // 자율 기능 또는 기록 중에만 동작
            if (current_mode_ != VehicleMode::STANDBY && current_mode_ != VehicleMode::MANUAL) {
                // 취소 후 MANUAL 상태로 돌아가는 것이 안전
                switch_mode(VehicleMode::MANUAL);
            }
        }
    }

    void aux_command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Aux command received: %d", msg->data);
        auto frame = can_msgs::msg::Frame();
        frame.id = 0x102; // 보조 기능 CAN ID
        frame.dlc = 1;
        frame.data[0] = msg->data;
        can_tx_publisher_->publish(frame);
    }


    void can_rx_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        // ID가 0x201 (STS_WHEEL_SPEEDS)인 메시지만 처리
        if (msg->id == 0x201 && msg->dlc == 8) {
            int32_t left_rpm, right_rpm;
            // Little-Endian 바이트 배열을 int32로 변환 (언패킹)
            // C++의 std::memcpy를 사용하여 안전하게 메모리 복사
            std::memcpy(&left_rpm, &msg->data[0], sizeof(int32_t));
            std::memcpy(&right_rpm, &msg->data[4], sizeof(int32_t));

            // std_msgs::msg::Int64 메시지 생성 및 발행
            // (ros2_odometry_estimation은 Int64를 받으므로 타입 캐스팅 필요)
            //auto left_rpm_msg = std_msgs::msg::Int64();
            //left_rpm_msg.data = static_cast<int64_t>(left_rpm_val);
            //left_rpm_publisher_->publish(left_rpm_msg);

            //auto right_rpm_msg = std_msgs::msg::Int64();
            //right_rpm_msg.data = static_cast<int64_t>(right_rpm_val);
            //right_rpm_publisher_->publish(right_rpm_msg);

	        calculate_and_publish_odometry(left_rpm, right_rpm);

            // (디버깅용 로그)
            RCLCPP_DEBUG(this->get_logger(), "Parsed RPM -> Left: %d, Right: %d",
                             left_rpm, right_rpm);

        }
        // TODO: ID 0x300 (긴급 정지) 등 다른 CAN ID 처리 로직 추가
    }


    // --- 유틸리티 함수 ---
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
    
    void publish_vehicle_status()
    {
        auto status_msg = interfaces::msg::VehicleStatus();
        status_msg.mode = to_string(current_mode_);
        // TODO: /from_can_bus에서 받은 현재 속도를 여기에 채워넣기
        // status_msg.current_velocity.linear.x = ...
        vehicle_status_publisher_->publish(status_msg);
    }

    void switch_mode(VehicleMode new_mode) {
        if (current_mode_ != new_mode) {
            RCLCPP_INFO(this->get_logger(), "Switching mode from %s to %s",
                        to_string(current_mode_).c_str(), to_string(new_mode).c_str());
            current_mode_ = new_mode;
	    latest_linear_vel_ratio_ = 0.0;
            latest_angular_vel_ratio_ = 0.0;
        }
    }

    void calculate_and_publish_odometry(int32_t left_rpm, int32_t right_rpm)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_odom_time_).seconds();
        last_odom_time_ = current_time;

        if (dt <= 0.0) return; // 시간 변화가 없으면 계산하지 않음

        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double track_width = this->get_parameter("track_width").as_double();

        double v_left = (left_rpm / 60.0) * (2 * M_PI * wheel_radius);
        double v_right = (right_rpm / 60.0) * (2 * M_PI * wheel_radius);

        double v = (v_right + v_left) / 2.0;
        double w = (v_right - v_left) / track_width;

        odom_x_ += v * std::cos(odom_theta_) * dt;
        odom_y_ += v * std::sin(odom_theta_) * dt;
        odom_theta_ += w * dt;

        // --- Odometry 메시지 발행 ---
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
    
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;
        odom_wheel_publisher_->publish(odom_msg);

        // --- TF 브로드캐스팅 ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = odom_x_;
        t.transform.translation.y = odom_y_;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<CoreControllerNode>(options));
    rclcpp::shutdown();
    return 0;
}
