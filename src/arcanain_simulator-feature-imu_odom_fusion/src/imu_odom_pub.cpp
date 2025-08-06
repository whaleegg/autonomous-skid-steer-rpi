#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMUメッセージを使用するために追加
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class ImuOdomPubNode : public rclcpp::Node
{
public:
  ImuOdomPubNode()
  : Node("imu_odom_pub_node")
  {
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("odom_path", 50);
    localmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
    laser_range_pub = this->create_publisher<visualization_msgs::msg::Marker>("laser_range_marker", 10);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // cmd_velサブスクライバを追加
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ImuOdomPubNode::cmd_vel_callback, this, std::placeholders::_1));

    // IMUデータをサブスクライブ
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data_raw", 10, std::bind(&ImuOdomPubNode::imu_callback, this, std::placeholders::_1));

    x = 0.0;
    y = 0.0;
    yaw = 0.0;  // 初期値をセット

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    path.header.frame_id = "odom";  // パスのフレームIDを設定

    timer_ = this->create_wall_timer(50ms, std::bind(&ImuOdomPubNode::timer_callback, this));

    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx = msg->linear.x;
    vth = msg->angular.z;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMUデータからクォータニオンを取得
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);

    // クォータニオンをオイラー角に変換してヨー角を取得
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    // ヨー角を更新
    yaw = yaw_tmp;
  }

  void timer_callback()
  {
    current_time = this->get_clock()->now();

    double dt = (current_time - last_time).seconds();
    double delta_x = vx * cos(yaw) * dt;  // yawはIMUから取得した値を使用
    double delta_y = vx * sin(yaw) * dt;

    x += delta_x;
    y += delta_y;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, yaw);  // IMUから取得したヨー角を使用
    geometry_msgs::msg::Quaternion odom_quat_msg = tf2::toMsg(odom_quat);

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;

    odom_broadcaster->sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat_msg;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub->publish(odom);

    // パスに現在の位置を追加
    geometry_msgs::msg::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.orientation = odom_quat_msg;
    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    path_pub->publish(path);

    // ローカルマップをパブリッシュ
    auto map = nav_msgs::msg::OccupancyGrid();
    map.header.stamp = current_time;
    map.header.frame_id = "base_link";
    map.info.resolution = 0.1;
    map.info.width = 40;
    map.info.height = 40;
    map.info.origin.position.x = -(map.info.height * map.info.resolution) / 2.0;
    map.info.origin.position.y = -(map.info.width * map.info.resolution) / 2.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);
    localmap_pub->publish(map);

    // レーザーのレンジを示すマーカーをパブリッシュ
    visualization_msgs::msg::Marker line_strip_marker;
    line_strip_marker.header.frame_id = "base_link";
    line_strip_marker.header.stamp = current_time;
    line_strip_marker.ns = "circle_line";
    line_strip_marker.id = 1;
    line_strip_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip_marker.action = visualization_msgs::msg::Marker::ADD;
    line_strip_marker.pose.orientation.w = 1.0;
    line_strip_marker.scale.x = 0.05;
    line_strip_marker.color.r = 1.0;
    line_strip_marker.color.g = 0.0;
    line_strip_marker.color.b = 0.0;
    line_strip_marker.color.a = 1.0;

    double radius = 1.0;
    int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
      double angle = i * 2.0 * M_PI / num_points;
      geometry_msgs::msg::Point p;
      p.x = radius * cos(angle);
      p.y = radius * sin(angle);
      p.z = 0.0;
      line_strip_marker.points.push_back(p);
    }

    laser_range_pub->publish(line_strip_marker);

    last_time = current_time;
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
    static_transform_stamped.header.frame_id = "map";
    static_transform_stamped.child_frame_id = "odom";
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.0;
    static_transform_stamped.transform.rotation.x = 0.0;
    static_transform_stamped.transform.rotation.y = 0.0;
    static_transform_stamped.transform.rotation.z = 0.0;
    static_transform_stamped.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(static_transform_stamped);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr localmap_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_range_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path;
  double x, y, yaw, vx, vth;
  rclcpp::Time current_time, last_time;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdomPubNode>());
  rclcpp::shutdown();
  return 0;
}

