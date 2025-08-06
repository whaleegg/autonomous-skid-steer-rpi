#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "yaml-cpp/yaml.h" // Ä¶¸®ºê·¹ÀÌ¼Ç ½Ã ÇÊ¿ä

// Ä¶¸®ºê·¹ÀÌ¼Ç µ¥ÀÌÅÍ¸¦ ÀúÀåÇÏ±â À§ÇÑ ±¸Á¶Ã¼
struct AxisCalibration {
    double center = 0.0;
    double deadzone_min = 0.0;
    double deadzone_max = 0.0;
    double min = -1.0;
    double max = 1.0;
};

class TeleopNode : public rclcpp::Node
{
public:
//    TeleopNode() : Node("teleop_node")
    explicit TeleopNode(const rclcpp::NodeOptions & options) : Node("teleop_node", options)
    {
        // === 1. ÆÄ¶ó¹ÌÅÍ ¼±¾ğ ===
        this->declare_parameter<std::string>("calibration_file", "/root/ros_ws/src/joystick_cal.yaml");
        this->declare_parameter<double>("expo_factor", 0.7);
        this->declare_parameter<int>("axis_linear", 1);
        this->declare_parameter<int>("axis_angular", 2);
        this->declare_parameter<double>("scale_linear", 1.0);
        this->declare_parameter<double>("scale_angular", 1.0);


        // === 2. Ä¶¸®ºê·¹ÀÌ¼Ç ÆÄÀÏ ·Îµå ===
        load_calibration_file();

        // === Publisherµé ¼±¾ğ ===
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
        mode_request_pub_ = this->create_publisher<std_msgs::msg::String>("/mode_request", 10);
        aux_command_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/aux_command", 10);

        // === Subscriber ¼±¾ğ ===
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));

        // === ¹öÆ° »óÅÂ ÀúÀåÀ» À§ÇÑ º¯¼ö ===
        // ÀÌÀü ¹öÆ° »óÅÂ¸¦ ÀúÀåÇÏ¿©, ¹öÆ°ÀÌ '´­¸®´Â ¼ø°£'¸¸ °¨ÁöÇÏ±â À§ÇÔ
        last_buttons_.resize(12, 0); // 12°³ ¹öÆ° °ø°£ È®º¸ (PS2 ÄÁÆ®·Ñ·¯ ±âÁØ)

        RCLCPP_INFO(this->get_logger(), "Teleop Node has been started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // --- 1. ½ºÆ½(Axes) Ã³¸® ---
        size_t axis_linear_idx = this->get_parameter("axis_linear").as_int(); 
        size_t axis_angular_idx = this->get_parameter("axis_angular").as_int();

        if (msg->axes.size() > axis_linear_idx && msg->axes.size() > axis_angular_idx)
        {
            // 3´Ü°è º¸Á¤ ÆÄÀÌÇÁ¶óÀÎ Àû¿ë
            double linear_val = normalize_axis(msg->axes[axis_linear_idx], axis_linear_idx);
            double angular_val = normalize_axis(msg->axes[axis_angular_idx], axis_angular_idx);
            
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = this->get_parameter("scale_linear").as_double() * linear_val;
            twist_msg.angular.z = this->get_parameter("scale_angular").as_double() * angular_val;
            cmd_vel_pub_->publish(twist_msg);
        }

        // --- 2. ¹öÆ°(Buttons) Ã³¸® ---
        if (msg->buttons.size() >= 12)
        {
            // O ¹öÆ° (1¹ø): ½Ãµ¿ On/Off
            if (is_button_pressed(msg, 1)) {
                publish_mode_request("toggle_manual");
            }
            // ¤± ¹öÆ° (2¹ø): ÀÚÀ² ÁÖÂ÷
            if (is_button_pressed(msg, 2)) {
                publish_mode_request("request_parking");
            }
            // ¡â ¹öÆ° (3¹ø): °æ·Î ±â·Ï/º¹±Í
            if (is_button_pressed(msg, 3)) {
                publish_mode_request("request_path_record_or_return");
            }
            // X ¹öÆ° (0¹ø): Ãë¼Ò
            if (is_button_pressed(msg, 0)) {
                publish_mode_request("request_cancel");
            }
            
            // L1 ¹öÆ° (buttons[4]) ´­¸²(rising edge) °¨Áö ½Ã bit 0 Åä±Û
            if (msg->buttons[4] == 1 && last_button_state_[0] == 0) {
                aux_cmd ^= (1 << 0);
            }
            // R1 ¹öÆ° (buttons[5]) ´­¸²(rising edge) °¨Áö ½Ã bit 1 Åä±Û
            if (msg->buttons[5] == 1 && last_button_state_[1] == 0) {
                aux_cmd ^= (1 << 1);
            }
            // ¹öÆ° »óÅÂ ÀúÀå (for next rising-edge detection)
            last_button_state_[0] = msg->buttons[4];
            last_button_state_[1] = msg->buttons[5];
            // »óÅÂ°¡ ¹Ù²î¾úÀ¸¸é publish
            if (aux_cmd != last_aux_cmd_) {
                auto aux_msg = std_msgs::msg::UInt8();
                aux_msg.data = aux_cmd;
                aux_command_pub_->publish(aux_msg);
                last_aux_cmd_ = aux_cmd;
            }
            // L1/R1 ¹öÆ° (4, 5¹ø): ±ôºıÀÌ
            // (±ôºıÀÌ´Â ´©¸£°í ÀÖ´Â µ¿¾È¸¸ ÄÑÁöµµ·Ï ´Ü¼øÇÏ°Ô ±¸Çö)
            //uint8_t aux_cmd = 0;
            //if (msg->buttons[4] == 1) { // L1
            //    aux_cmd |= (1 << 0); // bit 0: left blinker
            //}
            //if (msg->buttons[5] == 1) { // R1
            //    aux_cmd |= (1 << 1); // bit 1: right blinker
            //}
            //
            //// ?„ì¬ ê³„ì‚°??aux_cmdê°€ ?´ì „??ë°œí–‰?ˆë˜ ê°’ê³¼ ?¤ë? ê²½ìš°?ë§Œ ë°œí–‰
            //if (aux_cmd != last_aux_cmd_) {
            //    auto aux_msg = std_msgs::msg::UInt8();
            //    aux_msg.data = aux_cmd;
            //    aux_command_pub_->publish(aux_msg);
            //    
            //    // ë§ˆì?ë§‰ìœ¼ë¡?ë°œí–‰??ê°’ì„ ?„ì¬ ê°’ìœ¼ë¡??…ë°?´íŠ¸
            //    last_aux_cmd_ = aux_cmd;
            //}
        }

        // ÇöÀç ¹öÆ° »óÅÂ¸¦ ´ÙÀ½ Äİ¹éÀ» À§ÇØ ÀúÀå
        last_buttons_ = msg->buttons;
    }
    
    double normalize_axis(double raw_value, size_t axis_index)
    {
        if (calibration_data_.find(axis_index) == calibration_data_.end()) {
            return raw_value;
        }

        const auto& cal = calibration_data_.at(axis_index);

        if (raw_value > cal.deadzone_min && raw_value < cal.deadzone_max) {
            return 0.0;
        }

        double normalized_value = 0.0;
        if (raw_value >= cal.deadzone_max) {
            double effective_range = cal.max - cal.deadzone_max;
            if (effective_range > 1e-6)
                normalized_value = (raw_value - cal.deadzone_max) / effective_range;
            else
                normalized_value = 1.0;
        } else {
            double effective_range = cal.deadzone_min - cal.min;
            if (effective_range > 1e-6)
                normalized_value = (raw_value - cal.deadzone_min) / effective_range;
            else
                normalized_value = -1.0;
        }
        normalized_value = std::max(-1.0, std::min(normalized_value, 1.0));

        return apply_response_curve(normalized_value);
    }
    
    double apply_response_curve(double value)
    {
        double expo_factor = this->get_parameter("expo_factor").as_double();
        return expo_factor * std::pow(value, 3) + (1.0 - expo_factor) * value;
    }

    void load_calibration_file()
    {
        std::string file_path = this->get_parameter("calibration_file").as_string();
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            const auto& axes = config["axes"];
            for (const auto& axis_node : axes) {
                int axis_index = axis_node.first.as<int>();
                const auto& cal = axis_node.second;
                calibration_data_[axis_index] = {
                    cal["center"].as<double>(),
                    cal["deadzone_min"].as<double>(),
                    cal["deadzone_max"].as<double>(),
                    cal["min"].as<double>(),
                    cal["max"].as<double>()
                };
            }
            RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration file from %s", file_path.c_str());
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or parse calibration file %s: %s. Using raw values.", file_path.c_str(), e.what());
        }
    }
    
    // ¹öÆ°ÀÌ '¹æ±İ ´­·È´ÂÁö' È®ÀÎÇÏ´Â ÇïÆÛ ÇÔ¼ö
    bool is_button_pressed(const sensor_msgs::msg::Joy::SharedPtr& msg, int index)
    {
        return msg->buttons[index] == 1 && last_buttons_[index] == 0;
    }
    
    // ¸ğµå ¿äÃ» ¸Ş½ÃÁö¸¦ ¹ßÇàÇÏ´Â ÇïÆÛ ÇÔ¼ö
    void publish_mode_request(const std::string& request)
    {
        auto mode_msg = std_msgs::msg::String();
        mode_msg.data = request;
        mode_request_pub_->publish(mode_msg);
        RCLCPP_INFO(this->get_logger(), "Published mode request: %s", request.c_str());
    }

    // --- ¸â¹ö º¯¼ö ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_request_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr aux_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    std::vector<int> last_buttons_;
    std::map<int, AxisCalibration> calibration_data_; // Ä¶¸®ºê·¹ÀÌ¼Ç µ¥ÀÌÅÍ ÀúÀå¿ë map

    uint8_t last_aux_cmd_ = 0; // ?´ì „ aux_command ?íƒœ ?€??
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<TeleopNode>());
    // NodeOptionsë¥??ì„±?˜ê³ , ?Œë¼ë¯¸í„°ë¥??ë™?¼ë¡œ ? ì–¸?˜ë„ë¡??¤ì •
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    // ?˜ì •???ì„±?ì— options ?„ë‹¬
    rclcpp::spin(std::make_shared<TeleopNode>(options));
    rclcpp::shutdown();
    return 0;
}
