#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using Path = nav_msgs::msg::Path;

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher()
    : Node("path_publisher") {
	// 1. 파라미터 선언: 재발행 주기(Hz)를 Launch 파일에서 조절 가능하게 함
        this->declare_parameter<double>("publish_frequency", 1.0); // 기본값: 1Hz (1초에 한 번)
        double publish_period_sec = 1.0 / this->get_parameter("publish_frequency").as_double();

	// 2. Subscriber 생성: 부드러워진 경로를 입력받음
        //    QoS는 transient_local로 설정하여, 나중에 접속해도 경로를 받을 수 있도록 함
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        smoothed_path_sub_ = this->create_subscription<Path>(
            "/smoothed_path", qos_profile, std::bind(&PathPublisher::path_callback, this, std::placeholders::_1));

	// 3. Publisher 생성: pure_pursuit에 경로를 주기적으로 전달
        target_path_pub_  = this->create_publisher<Path>("tgt_path", 10);
        visualize_path_pub_ = this->create_publisher<Path>("visualize_tgt_path", 10);

	// 4. Timer 생성: 주기적인 발행을 위함
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_period_sec),
            std::bind(&PathPublisher::publish_path_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Path Publisher Node has been started.");

        //// ホームディレクトリのパスを取得
        //std::string home_dir = getenv("HOME");
        //// ファイルのフルパスを組み立てる
        //std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/simulation_path.csv";
        ////loadPathData(file_path);
        //timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(100), std::bind(&PathPublisher::publishPath, this));
    }

private:
    //void loadPathData(const std::string& file_path) {
    //    std::ifstream file(file_path);
    //    if (!file.is_open()) {
    //        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
    //        return;
    //    }

    //    std::string line;
    //    std::getline(file, line); // Skip the header

    //    int pose_count = 0; // Add a counter for poses

    //    while (std::getline(file, line)) {
    //        std::stringstream ss(line);
    //        std::string data;
    //        std::vector<std::string> row_data;

    //        while (std::getline(ss, data, ',')) {
    //            row_data.push_back(data);
    //        }

    //        geometry_msgs::msg::PoseStamped pose;
    //        pose.header.stamp = this->now();
    //        pose.header.frame_id = "map";
    //        pose.pose.position.x = std::stod(row_data[0]);
    //        pose.pose.position.y = std::stod(row_data[1]);
    //        pose.pose.position.z = std::stod(row_data[2]);
    //        pose.pose.orientation.x = std::stod(row_data[3]);
    //        pose.pose.orientation.y = std::stod(row_data[4]);
    //        pose.pose.orientation.z = std::stod(row_data[5]);
    //        pose.pose.orientation.w = std::stod(row_data[6]);
    //        path_.poses.push_back(pose);

    //        // 視覚用
    //        pose.pose.position.z = 0.0;
    //        visualize_path_.poses.push_back(pose);

    //        pose_count++; // Increment the counter
    //    }

    //    path_.header.stamp = this->now();
    //    path_.header.frame_id = "map";
    //    visualize_path_.header.stamp = this->now();
    //    visualize_path_.header.frame_id = "map";

    //    RCLCPP_INFO(this->get_logger(), "Loaded %d poses from the CSV file.", pose_count); // Log the number of loaded poses
    //}

    //void publishPath() {
    //    path_.header.stamp = this->now();
    //    path_.header.frame_id = "map";
    //    visualize_path_.header.stamp = this->now();
    //    visualize_path_.header.frame_id = "map";

    //    path_pub_->publish(path_);
    //    visualize_path_pub_->publish(visualize_path_);
    //}

    void path_callback(const Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "New smoothed path received with %zu points to republish.", msg->poses.size());
        // 수신된 경로를 내부 변수에 저장
        current_path_ = *msg;
        has_path_ = true;
    }

    void publish_path_timer_callback()
    {
        // 저장된 경로가 있을 경우에만 주기적으로 발행
        if (has_path_) {
            // 타임스탬프를 현재 시간으로 업데이트하여 최신 정보임을 알림
            current_path_.header.stamp = this->get_clock()->now();
            target_path_pub_->publish(current_path_);
        }
    }

    rclcpp::Subscription<Path>::SharedPtr smoothed_path_sub_;
    rclcpp::Publisher<Path>::SharedPtr target_path_pub_;
    rclcpp::Publisher<Path>::SharedPtr visualize_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Path visualize_path_;

    bool has_path_ = false;
    Path current_path_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
