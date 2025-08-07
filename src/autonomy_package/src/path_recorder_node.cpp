// src/autonomy_package/src/path_recorder_node.cpp

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/vehicle_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathRecorderNode : public rclcpp::Node
{
public:
    PathRecorderNode() : Node("path_recorder_node")
    {
        // 최종 경로 전달용: Latch-like. 마지막 메시지를 보관.
        auto latch_like_qos = rclcpp::QoS(1).transient_local().reliable();
        // 실시간 시각화용: Best-effort. 최신 데이터만 빠르게 전달.
        auto sensor_data_qos = rclcpp::QoS(1).best_effort();

        status_sub_ = this->create_subscription<interfaces::msg::VehicleStatus>(
            "/vehicle_status", 10, std::bind(&PathRecorderNode::status_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10, std::bind(&PathRecorderNode::odom_callback, this, std::placeholders::_1));

        // 최종 경로 발행기 (path_return_node가 사용)
        recorded_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/recorded_path", latch_like_qos);
        
        // 실시간 시각화용 경로 발행기 (RViz2가 사용)
        path_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>("/recording_path_viz", 10);
        
        RCLCPP_INFO(this->get_logger(), "Path Recorder Node has been started.");
    }

private:
    bool is_recording_ = false;
    nav_msgs::msg::Path recorded_path_;
    
    rclcpp::Subscription<interfaces::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recorded_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;

    void status_callback(const interfaces::msg::VehicleStatus::SharedPtr msg)
    {
        // core_controller_node로부터 현재 모드를 전달받음
        std::string current_mode = msg->mode;

        if (current_mode == "RECORDING" && !is_recording_) {
            // 기록 시작
            RCLCPP_INFO(this->get_logger(), "Start path recording.");
            recorded_path_.poses.clear();
            recorded_path_.header.frame_id = "odom";
            is_recording_ = true;
        } else if (current_mode != "RECORDING" && is_recording_) {
            // 기록 중지 시, 최종 완성된 경로를 한 번 발행
            RCLCPP_INFO(this->get_logger(), "Stop path recording. Publishing final path with %zu points.", recorded_path_.poses.size());
            recorded_path_.header.stamp = this->get_clock()->now();
            recorded_path_pub_->publish(recorded_path_);
            is_recording_ = false;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 기록 모드일 때만 경로 저장
        if (is_recording_) {
            geometry_msgs::msg::PoseStamped current_pose;
            current_pose.header = msg->header;
            current_pose.pose = msg->pose.pose;
            
            // TODO: 너무 촘촘하게 저장되지 않도록, 이전 포인트와 일정 거리 이상일 때만 추가하는 로직 추가
            recorded_path_.poses.push_back(current_pose);

	    // --- [수정] 실시간 시각화는 path_viz_pub_으로 발행 ---
            recorded_path_.header.stamp = this->get_clock()->now();
            path_viz_pub_->publish(recorded_path_);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
