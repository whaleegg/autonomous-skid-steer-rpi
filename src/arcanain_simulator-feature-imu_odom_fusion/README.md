<p style="display: inline">
  <!-- Programming Language -->
  <img src="https://img.shields.io/badge/-C++-00599C.svg?logo=c%2B%2B&style=for-the-badge">
  <!-- ROS 2 -->
  <img src="https://img.shields.io/badge/-ROS%202-22314E.svg?logo=ros&style=for-the-badge&logoColor=white">
  <!-- Geometry Messages -->
  <img src="https://img.shields.io/badge/-Geometry%20Messages-7F7F7F.svg?logo=ros&style=for-the-badge&logoColor=white">
  <!-- Navigation Messages -->
  <img src="https://img.shields.io/badge/-Navigation%20Messages-7F7F7F.svg?logo=ros&style=for-the-badge&logoColor=white">
  <!-- TF2 -->
  <img src="https://img.shields.io/badge/-TF2-7F7F7F.svg?logo=ros&style=for-the-badge&logoColor=white">
</p>

## Functional Overview

## Requirements
### System Requirements
- OS : Ubuntu 22.04  
- ROS2 : Humble

## How To Use
### Execution Steps
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 launch arcanain_simulator simulator.py
```

### Folder Structure

## Interface Table

### Input

### Output

### Internal Values

## Software architecture

### Class Diagram

```mermaid
classDiagram
    class Obstacle {
        double x
        double y
        double radius
        double margin
    }
    class ObstaclePublisher {
        -std::vector<Obstacle> obstacles
        -double base_x
        -double base_y
        -rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_obstacle_pub
        -rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_pub
        -rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub
        -rclcpp::TimerBase::SharedPtr timer_
        +ObstaclePublisher()
        -void obstacle_publish()
        -void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr)
    }
    ObstaclePublisher --|> Obstacle : Uses

    class OdometryPublisher {
        -rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub
        -rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub
        -rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr localmap_pub
        -rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_range_pub
        -rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber
        -std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster
        -std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_
        -rclcpp::TimerBase::SharedPtr timer_
        -nav_msgs::msg::Path path
        -double x, y, th, vx, vth
        -rclcpp::Time current_time, last_time
        +OdometryPublisher()
        -void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr)
        -void timer_callback()
        -void send_static_transform()
    }

    class WaypointNavigation {
        -rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub
        -rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub
        -std::vector<geometry_msgs::msg::PoseStamped> waypoints
        -int current_waypoint_index
        +WaypointNavigation()
        -void odom_callback(const nav_msgs::msg::Odometry::SharedPtr)
        -geometry_msgs::msg::PoseStamped create_waypoint(double, double, double)
    }
```

### Flowchart

## Functional Requirements

## Detailed Design
