#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16.hpp"
#include <cstdio>

class CmdVelListenerNode : public rclcpp::Node {
public:
    CmdVelListenerNode() : Node("cmd_vel_listener_node") {
        // Subscriber 선언 및 초기화
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelListenerNode::cmd_vel_callback, this, std::placeholders::_1));

        // Publisher 선언 및 초기화
        lwheel_publisher_ = this->create_publisher<std_msgs::msg::Int16>("lwheel", 10);
        rwheel_publisher_ = this->create_publisher<std_msgs::msg::Int16>("rwheel", 10);
    
        lwheel_count_ = 0;
        rwheel_count_ = 0;
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
        // Twist 메시지의 값을 출력
        /*
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel message - Linear: %.2f, %.2f, %.2f, Angular: %.2f, %.2f, %.2f",
                    cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->linear.z,
                    cmd_vel_msg->angular.x, cmd_vel_msg->angular.y, cmd_vel_msg->angular.z);
        */
        
        // l_wheel 및 r_wheel 토픽으로 각Int16();각 linear.x 및 angular.z 값을 Int16 메시지로 퍼블리시
        auto lwheel_msg = std_msgs::msg::Int16();
        auto rwheel_msg = std_msgs::msg::Int16();
        if (cmd_vel_msg->linear.x == 0.5 && cmd_vel_msg->angular.z == 0) {
            lwheel_count_ += 1;  
            rwheel_count_ += 1;
        }
        else if (cmd_vel_msg->linear.x == 0 && cmd_vel_msg->angular.z == 0) {
            lwheel_count_ -= 1;  
            rwheel_count_ -= 1;
        }
        else if (cmd_vel_msg->linear.x == 0 && cmd_vel_msg->angular.z == 1) {
            rwheel_count_ += 1;
        }
        else if (cmd_vel_msg->linear.x == 0 && cmd_vel_msg->angular.z == -1) {
            lwheel_count_ += 1;
        }

        lwheel_msg.data = lwheel_count_;
        rwheel_msg.data = rwheel_count_;
        lwheel_publisher_->publish(lwheel_msg);
        rwheel_publisher_->publish(rwheel_msg);
        RCLCPP_INFO(this->get_logger(), "rwheel_encoder_count: %d lwheel_encoder_count: %d",rwheel_count_ ,lwheel_count_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr lwheel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rwheel_publisher_;

    int16_t lwheel_count_;
    int16_t rwheel_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelListenerNode>());
    rclcpp::shutdown();
    return 0;
}