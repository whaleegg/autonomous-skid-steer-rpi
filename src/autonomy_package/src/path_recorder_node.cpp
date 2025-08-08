// src/autonomy_package/src/path_recorder_node.cpp

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/vehicle_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>

class PathRecorderNode : public rclcpp::Node
{
public:
    PathRecorderNode() : Node("path_recorder_node")
    {
	// Path용 QoS 프로파일 (Latching)
	rclcpp::QoS path_qos_profile(1);
        path_qos_profile.transient_local(); // Path 토픽은 마지막 메시지를 유지하는 것이 좋음

	// 일반 데이터용 QoS 프로파일
        rclcpp::QoS data_qos_profile(10);

        // 1. 파라미터 선언
        this->declare_parameter<double>("min_distance_threshold", 0.05); // 5cm

        // 2. Subscriber 선언
        // core_controller가 transient_local로 발행한다면, 여기도 맞춰줘야 함
        status_sub_ = this->create_subscription<interfaces::msg::VehicleStatus>(
            "/vehicle_status", path_qos_profile, std::bind(&PathRecorderNode::status_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/wheel", data_qos_profile, std::bind(&PathRecorderNode::odometry_callback, this, std::placeholders::_1));

        // 3. Publisher 선언
        recorded_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/recorded_path", path_qos_profile);
        visualized_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/visualized_path", path_qos_profile);

        RCLCPP_INFO(this->get_logger(), "Path Recorder Node has been started.");
    }

private:
    // === 콜백 함수들 ===

    void status_callback(const interfaces::msg::VehicleStatus::SharedPtr msg)
    {
        bool is_currently_recording = (msg->mode == "RECORDING");

        if (is_currently_recording && !is_recording_) {
            // --- RECORDING 모드 시작 감지 ---
            RCLCPP_INFO(this->get_logger(), "Start path recording.");
            is_recording_ = true;
            // 이전 경로 데이터를 깨끗하게 지움
            recorded_path_.poses.clear();
            recorded_path_.header.frame_id = "odom"; // 경로의 기준 좌표계 설정
            // 현재 위치를 경로의 첫 번째 시작점으로 추가하기 위해 최신 오도메트리 저장
            latest_odom_ = nullptr;
        }
        else if (!is_currently_recording && is_recording_) {
            // --- RECORDING 모드 종료 감지 ---
            RCLCPP_INFO(this->get_logger(), "Stop path recording. Total points: %zu", recorded_path_.poses.size());
            is_recording_ = false;

            // 완성된 경로를 /recorded_path 토픽으로 한 번 발행
            if (!recorded_path_.poses.empty()) {
		// --- [핵심] 경로 뒤집기 ---
                std::reverse(recorded_path_.poses.begin(), recorded_path_.poses.end());
                RCLCPP_INFO(this->get_logger(), "Path has been reversed for return journey.");

                recorded_path_.header.stamp = this->get_clock()->now();
                recorded_path_publisher_->publish(recorded_path_);
            }
        }
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 최신 오도메트리 정보 저장
        latest_odom_ = msg;

        if (!is_recording_) {
            return; // 기록 중이 아니면 아무것도 하지 않음
        }

        auto current_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        current_pose->header = msg->header;
        current_pose->pose = msg->pose.pose;
        
        // 경로의 첫 번째 포인트를 추가
        if (recorded_path_.poses.empty()) {
            recorded_path_.poses.push_back(*current_pose);
            RCLCPP_INFO(this->get_logger(), "Added first point to the path.");
        }
        // 마지막 지점과의 거리가 임계값 이상일 때만 새 포인트 추가
        else if (get_distance(recorded_path_.poses.back().pose.position, current_pose->pose.position) > this->get_parameter("min_distance_threshold").as_double())
        {
            recorded_path_.poses.push_back(*current_pose);
            // 시각화를 위해 현재까지의 경로를 발행
            visualize_path();
        }
    }
    
    // === 유틸리티 함수 ===

    void visualize_path()
    {
        if (!recorded_path_.poses.empty()) {
            recorded_path_.header.stamp = this->get_clock()->now();
            visualized_path_publisher_->publish(recorded_path_);
        }
    }

    double get_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // --- 멤버 변수 ---
    bool is_recording_ = false;
    nav_msgs::msg::Path recorded_path_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_ = nullptr; // 최신 오도메트리 저장을 위함

    rclcpp::Subscription<interfaces::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recorded_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visualized_path_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
