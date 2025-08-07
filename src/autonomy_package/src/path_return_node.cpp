// autonomy_package/src/path_return_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/vehicle_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include <algorithm> // std::reverse

class PathReturnNode : public rclcpp::Node
{
public:
    PathReturnNode() : Node("path_return_node")
    {
	auto latch_like_qos = rclcpp::QoS(1).transient_local().reliable();

        // --- [핵심 수정 2] 콜백 함수와 멤버 변수 선언에서도 모두 수정 ---
        status_sub_ = this->create_subscription<interfaces::msg::VehicleStatus>(
            "/vehicle_status", 10, std::bind(&PathReturnNode::status_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/recorded_path", latch_like_qos, std::bind(&PathReturnNode::path_callback, this, std::placeholders::_1));

        target_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/tgt_traj", latch_like_qos);

        RCLCPP_INFO(this->get_logger(), "Path Return Node has been started.");
    }

private:
    bool is_returning_ = false;
    nav_msgs::msg::Path received_path_;
    bool path_published_to_pursuit_ = false;

    rclcpp::Subscription<interfaces::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_trajectory_pub_;

    void status_callback(const interfaces::msg::VehicleStatus::SharedPtr msg)
    {
        std::string current_mode = msg->mode;
        
        if (current_mode == "RETURNING" && !is_returning_) {
            // 복귀 모드로 처음 진입
            is_returning_ = true;
            path_published_to_pursuit_ = false; // 경로 재발행을 위해 플래그 리셋
            RCLCPP_INFO(this->get_logger(), "Start path returning.");
        } else if (current_mode != "RETURNING" && is_returning_) {
            // 복귀 모드 종료
            is_returning_ = false;
        }

        // 복귀 모드이고, 아직 경로를 발행하지 않았고, 되돌아갈 경로가 있다면
        if (is_returning_ && !path_published_to_pursuit_ && !received_path_.poses.empty()) {
            target_trajectory_pub_->publish(received_path_);
            path_published_to_pursuit_ = true;
            RCLCPP_INFO(this->get_logger(), "Published return path to Pure Pursuit planner.");
        }
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received a path to return with %zu points.", msg->poses.size());
        received_path_ = *msg;
        // 경로를 역순으로 변환하여 저장
        std::reverse(received_path_.poses.begin(), received_path_.poses.end());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathReturnNode>());
    rclcpp::shutdown();
    return 0;
}
