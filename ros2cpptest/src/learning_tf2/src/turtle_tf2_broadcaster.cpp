#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FramePublisher : public rclcpp::Node
{
public:
FramePublisher()
: Node("turtle_tf2_frame_publisher")
{
    // 首先，我们定义并获取一个参数turtlename，它指定一个海龟名称，例如乌龟 1 或乌龟 2。
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    //之后，节点订阅主题turtleX/pose并对每条传入消息运行函数handle_turtle_pose。
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        topic_name, 10,
        std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
}

private:
void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
{
    //现在，我们创建一个 TransformStamped 对象并为其提供适当的元数据。
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
}

rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::string turtlename_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}