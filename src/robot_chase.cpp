#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class RobotChaseNode : public rclcpp::Node
{
public:
    RobotChaseNode() : Node("robot_chase_node")
    {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = create_wall_timer(100ms, std::bind(&RobotChaseNode::controlLoop, this));
        kp_distance_ = 0.5;
        kp_yaw_ = 1.0;
        distance_threshold_ = 0.4;  
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    double kp_distance_;
    double kp_yaw_;
    double distance_threshold_;  

    void controlLoop()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform("rick/base_link", "morty/base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        double error_distance = sqrt(pow(transform.transform.translation.x, 2) + pow(transform.transform.translation.y, 2));
        double error_yaw = atan2(transform.transform.translation.y, transform.transform.translation.x);
        double angular_velocity = kp_yaw_ * error_yaw;
        double linear_velocity = 0.0;

        if (error_distance > distance_threshold_)
        {
            linear_velocity = kp_distance_ * error_distance;
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_velocity;
        twist.angular.z = angular_velocity;
        publisher_->publish(twist);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotChaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}