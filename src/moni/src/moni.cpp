#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        // 创建发布器，用于发布MarkerArray消息对象
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);

        // 初始化pitch、yaw和速度值
        pitch_ = 1.0;
        yaw_ = 1.0;
        speed_ = 1.0;

        // 创建订阅器，用于接收新的pitch、yaw和速度值
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("trajectory_params", 10,
            std::bind(&TrajectoryNode::callback, this, std::placeholders::_1));

        // 创建定时器，每秒更新一次弹道
        timer_ = this->create_wall_timer(1s, std::bind(&TrajectoryNode::update_trajectory, this));

        // 初始化tf2变换
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void generate_trajectory()
    {
        // 创建MarkerArray消息对象
        auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

        // 创建线条标记...
        auto line_strip_marker = std::make_unique<visualization_msgs::msg::Marker>();
        line_strip_marker->header.frame_id = "base_frame";
        line_strip_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip_marker->action = visualization_msgs::msg::Marker::ADD;
        line_strip_marker->scale.x = 0.1;
        line_strip_marker->pose.orientation.w = 1.0;
        line_strip_marker->color.a = 1.0;
        line_strip_marker->color.r = 1.0;

        // 生成弹道点...
        const double GRAVITY = 9.78;
        const double k = 0.038;
        for (int t = 0; t <= 20; t++)
        {
            const double t0 = static_cast<double>(t) / 10.0;
            const double s = std::log(t0 * k * speed_ * std::cos(pitch_*3.14/180.0) + 1.0) / k;
            const double x = s * std::cos(yaw_*3.14/180.0);
            const double y = s * std::sin(yaw_*3.14/180.0);
            const double z = (speed_ * std::sin(pitch_*3.14/180.0) * t0 - GRAVITY * t0 * t0 / 2.0);
            RCLCPP_INFO_STREAM(this->get_logger(), "position :" << std::endl 
                                    << "x: " << x << std::endl 
                                    << "y: " << y << std::endl 
                                    << "z: " << z << std::endl 
                                    << "t0: " << t0 << std::endl 
                                    <<"speed_: " << speed_  << std::endl
                                    << "speed_z: " << speed_ * std::sin(pitch_*3.14/180.0) << std::endl);

            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            line_strip_marker->points.push_back(point);
        }

        // 将标记添加到MarkerArray消息对象中...
        marker_array_msg->markers.push_back(*line_strip_marker);

        publisher_->publish(std::move(marker_array_msg));
    }

    void update_trajectory()
    {
        // 调用generate_trajectory()函数重新生成弹道...
        generate_trajectory();

        // 发布tf2变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "base_frame";
        transform_stamped.child_frame_id = "trajectory_frame";
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 更新pitch、yaw和速度值...
        pitch_ = -msg->data[0];
        yaw_ = msg->data[1];
        speed_ = msg->data[2];
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double pitch_;
    double yaw_;
    double speed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}