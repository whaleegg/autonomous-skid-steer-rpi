#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
//#include <struct.h> // C++ struct.h for C-style packing (if needed)

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"
#include "interfaces/msg/vehicle_status.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
    CoreControllerNode() : Node("core_controller_node")
    {
	// === 1. 파라미터 선언 ===
        this->declare_parameter<double>("max_pwm", 255.0);
	this->declare_parameter<double>("steering_sensitivity", 0.7);
        this->declare_parameter<double>("linear_sensitivity", 1.0);
        // 향후 자율주행 모드에서 사용할 파라미터들
        this->declare_parameter<double>("max_linear_velocity", 0.8);
        this->declare_parameter<double>("max_angular_velocity", 2.0);

        // 2. 현재 모드를 STANDBY로 초기화
        current_mode_ = VehicleMode::STANDBY;

        // === 3. Publisher 및 Subscriber 선언 ===
        can_tx_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
        vehicle_status_publisher_ = this->create_publisher<interfaces::msg::VehicleStatus>("/vehicle_status", 10);

        manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", 10, std::bind(&CoreControllerNode::manual_cmd_callback, this, std::placeholders::_1));
        
        mode_request_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_request", 10, std::bind(&CoreControllerNode::mode_request_callback, this, std::placeholders::_1));

        aux_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/aux_command", 10, std::bind(&CoreControllerNode::aux_command_callback, this, std::placeholders::_1));

        //to store trj and return trj
        //odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //    "/odom", 10,
        //    std::bind(&Joy2CanNode::odomCallback, this, std::placeholders::_1));
        tgt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/tgt_path", 10);
        
        // TODO: 자율주행 노드가 발행할 토픽 구독
        // auto_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>( ... );
        // TODO: TC375의 피드백을 받을 토픽 구독
        // can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>( ... );

        // === 메인 제어 루프 타이머 ===
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&CoreControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Core Controller Node has been started in STANDBY mode.");
    }

private:
    // === 멤버 변수 ===
    VehicleMode current_mode_;
    double latest_linear_vel_ratio_ = 0.0;
    double latest_angular_vel_ratio_ = 0.0;
    // TODO: 자율주행용 속도 변수
    // double latest_auto_linear_vel_ = 0.0;
    // double latest_auto_angular_vel_ = 0.0;
    
    //to store trj
    //bool recording_{ false };
	std::vector<geometry_msgs::msg::PoseStamped> recorded_poses_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tgt_path_pub_;

    // --- ROS 2 인터페이스 객체 ---
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_publisher_;
    rclcpp::Publisher<interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr aux_command_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;



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
        //RCLCPP_INFO(this->get_logger(), "In MANUAL mode...", RCLC_LOG_THROTTLE(RCUTILS_STEADY_TIME, 1000));
	RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "In MANUAL mode..."
    	);
	handle_manual_mode_logic();
    }

    void handle_parking_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In PARKING mode...");
        // TODO: /cmd_vel_auto 토픽의 값을 PWM으로 변환하여 전송         
    }

    void handle_recording_mode()
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RECORDING mode...");
        // TODO: 여기에 경로를 기록하는 로직 호출
		//경로 기록을 위한 토픽 발행 또는 데이터 저장
        recorded_poses_.clear();
        //recording_ = true;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", 10, std::bind(&CoreControllerNode::odomCallback, this, std::placeholders::_1));
     
        // 기록 중에도 수동 제어는 계속되어야 하므로, manual 핸들러를 호출
        handle_manual_mode_logic(); // 기존 manual 핸들러의 내용을 별도 함수로 분리
    }

    void handle_returning_mode() {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RETURNING mode...");
        // TODO: /cmd_vel_auto 토픽의 값을 PWM으로 변환하여 전송
        //recording_ = false;
        generateReturnPath();
        imu_sub_.reset();
    }

    void handle_emergency_stop() {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In EMERGENCY_STOP mode!");
        // 모든 제어 명령 전송을 중단하거나, 명시적인 정지 명령을 보냄
        send_pwm_command(0, 0);
    }

    void handle_manual_mode_logic()
    {
        const double MAX_PWM = this->get_parameter("max_pwm").as_double();

        double left_signal = this->latest_linear_vel_ratio_ - this->latest_angular_vel_ratio_;
        double right_signal = this->latest_linear_vel_ratio_ + this->latest_angular_vel_ratio_;

        double max_abs_signal = std::max(std::abs(left_signal), std::abs(right_signal));
        if (max_abs_signal > 1.0) {
            left_signal /= max_abs_signal;
            right_signal /= max_abs_signal;
        }

        int left_pwm = static_cast<int>(left_signal * MAX_PWM);
        int right_pwm = static_cast<int>(right_signal * MAX_PWM);

        send_pwm_command(left_pwm, right_pwm); 
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

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom->header;
        ps.pose = odom->pose.pose;
        recorded_poses_.push_back(ps);
    }
    void generateReturnPath() {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom";
        // recorded_poses_ 에 담긴 순서를 **역순(reverse)** 으로 하여 돌아오는 경로 생성
        for (auto it = recorded_poses_.rbegin(); it != recorded_poses_.rend(); ++it) {
            path.poses.push_back(*it);
        }
        // tgt_path 퍼블리시
        tgt_path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published return path with %zu points", path.poses.size());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreControllerNode>());
    rclcpp::shutdown();
    return 0;
}
