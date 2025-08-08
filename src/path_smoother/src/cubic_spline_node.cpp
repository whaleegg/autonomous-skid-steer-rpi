#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream> // 追加
#include "path_smoother/cubic_spline_2d.hpp"  // CubicSpline2Dクラスのヘッダーファイルをインクルード
#include <vector>

//using std::placeholders::_1;
//using namespace std::chrono_literals;

using Path = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class CubicSplinePathPublisher : public rclcpp::Node
{
public:
    CubicSplinePathPublisher()
        : Node("cubic_spline_node")
    {
	// 1. 파라미터 선언
        this->declare_parameter<double>("interpolation_distance", 0.1); // 10cm 간격으로 보간

        //path_pub_ = this->create_publisher<nav_msgs::msg::Path>("tgt_path", 10);
        //timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(100), std::bind(&CubicSplinePathPublisher::publishPath, this));

	rclcpp::QoS qos_profile(1);
        qos_profile.transient_local(); // 경로 토픽은 Latching이 좋음

	// 2. Subscriber 생성: 원본 경로를 입력받음
        raw_path_sub_ = this->create_subscription<Path>(
            "/recorded_path", 10, std::bind(&CubicSplinePathPublisher::path_callback, this, std::placeholders::_1));

	// 3. Publisher 생성: 부드러워진 경로를 발행
        smoothed_path_pub_ = this->create_publisher<Path>("/smoothed_path", qos_profile);

	RCLCPP_INFO(this->get_logger(), "CubicSplinePathPublisher has been started.");
    }

//    void save_csv()
//    {
//        // ホームディレクトリのパスを取得
//        std::string home_dir = getenv("HOME");
//        // ファイルのフルパスを組み立てる
//        std::string file_path = home_dir + "/ros2_ws/src/path_smoother/path/simulation_path.csv";

//        // ファイルを開く
//        std::ofstream file(file_path);
//        if (!file.is_open()) {
//            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
//            return;
//        }

//        file << "x,y,z,w0,w1,w2,w3\n";

//        for (const auto& point : path_)
//        {
//            file << point[0] << "," << point[1] << "," << point[2] << ","
//                 << point[3] << "," << point[4] << "," << point[5] << "," << point[6] << "\n";
//        }

//        file.close();
//    }

//    // Spline interpolation results:
//    std::vector<double> rx_;  // x coordinates after spline interpolation
//    std::vector<double> ry_;  // y coordinates after spline interpolation
//    std::vector<double> ryaw_;  // yaw angles after spline interpolation
//    std::vector<double> rk_;  // curvatures after spline interpolation

//    bool save_flag = false;

private:
    //void publishPath()
    //{
    //    // target pathをpublish
    //    nav_msgs::msg::Path path_msg;
    //    path_msg.header.stamp = this->get_clock()->now();
    //    path_msg.header.frame_id = "map";

    //    for (size_t i = 0; i < rx_.size(); ++i)
    //    {
    //        geometry_msgs::msg::PoseStamped pose;
    //        pose.header = path_msg.header;
    //        pose.pose.position.x = rx_[i];
    //        pose.pose.position.y = ry_[i];
    //        pose.pose.position.z = rk_[i];
    //        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), ryaw_[i]));
    //        path_msg.poses.push_back(pose);
    //    }

    //    path_pub_->publish(path_msg);

    //    // pathをcsvファイルに保存
    //    for (size_t i = 0; i < path_msg.poses.size(); ++i)
    //    {
    //        std::vector<double> current_point = {
    //            path_msg.poses[i].pose.position.x,
    //            path_msg.poses[i].pose.position.y,
    //            path_msg.poses[i].pose.position.z,
    //            path_msg.poses[i].pose.orientation.x,
    //            path_msg.poses[i].pose.orientation.y,
    //            path_msg.poses[i].pose.orientation.z,
    //            path_msg.poses[i].pose.orientation.w
    //        };
    //        path_.push_back(current_point);
    //    }

    //    if (!save_flag) {
    //        save_csv();
    //        std::cout << "Save Path finished!" << std::endl;
    //    } 
        
    //    save_flag = true;
    //}

    //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    //rclcpp::TimerBase::SharedPtr timer_;
    //std::vector<std::vector<double>> path_;
//};


    void path_callback(const Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received path has less than 2 points. Cannot smooth.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received a path with %zu points to smooth.", msg->poses.size());

        // 1. 수신된 Path 메시지에서 x, y 좌표 벡터 추출
        std::vector<double> x_points, y_points;
        for (const auto& pose_stamped : msg->poses) {
            x_points.push_back(pose_stamped.pose.position.x);
            y_points.push_back(pose_stamped.pose.position.y);
        }

        // 2. Cubic Spline 알고리즘 실행
        CubicSpline2D spline_interpolator(x_points, y_points);
        double ds = this->get_parameter("interpolation_distance").as_double();

        Path smoothed_path_msg;
        smoothed_path_msg.header.stamp = this->get_clock()->now();
        smoothed_path_msg.header.frame_id = msg->header.frame_id; // 원본 경로와 동일한 프레임 사용

        // 3. 계산된 스플라인 경로를 Path 메시지로 변환
        for (double s = 0; s < spline_interpolator.get_s_max(); s += ds)
        {
            auto [ix, iy] = spline_interpolator.calc_position(s);
            double yaw = spline_interpolator.calc_yaw(s);
            double curvature = spline_interpolator.calc_curvature(s);

            PoseStamped pose;
            pose.header = smoothed_path_msg.header;
            pose.pose.position.x = ix;
            pose.pose.position.y = iy;
            pose.pose.position.z = curvature; // z 좌표에 곡률 정보를 저장 (pure_pursuit에서 사용)

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
            
            smoothed_path_msg.poses.push_back(pose);
        }

        // 4. 부드러워진 경로를 /smoothed_path 토픽으로 발행
        smoothed_path_pub_->publish(smoothed_path_msg);
        RCLCPP_INFO(this->get_logger(), "Published smoothed path with %zu points.", smoothed_path_msg.poses.size());
    }

    rclcpp::Subscription<Path>::SharedPtr raw_path_sub_;
    rclcpp::Publisher<Path>::SharedPtr smoothed_path_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubicSplinePathPublisher>();

    // std::vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    // std::vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    //std::vector<double> x = {0.0, 1.0, 1.5, 3.0, 4.0};
    //std::vector<double> y = {0.0, 1.0, 1.5, 1.0, 0.0};
    //double ds = 0.1;

    //CubicSpline2D sp(x, y);
    //std::vector<double> s_values;
    //for (double s = 0; s <= sp.get_s_max(); s += ds)
    //{
    //    s_values.push_back(s);
    //}

    //for (double s : s_values)
    //{
    //    auto [ix, iy] = sp.calc_position(s);
    //    node->rx_.push_back(ix);
    //    node->ry_.push_back(iy);
    //    node->ryaw_.push_back(sp.calc_yaw(s));
    //    node->rk_.push_back(sp.calc_curvature(s));
    //}

    rclcpp::spin(std::make_shared<CubicSplinePathPublisher>());
    rclcpp::shutdown();
    return 0;
}
