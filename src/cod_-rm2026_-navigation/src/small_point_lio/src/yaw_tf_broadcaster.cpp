#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>

class YawTFBroadcaster : public rclcpp::Node {
public:
    YawTFBroadcaster() : Node("yaw_tf_broadcaster") {
        // 装位固定参数——从 launch param 传,以后只改数值不改代码
        mount_x_     = declare_parameter("mount_x", 0.05289);        // 前后,待实测
        mount_y_     = declare_parameter("mount_y", 0.15191);        // 左右
        mount_z_     = declare_parameter("mount_z", 0.15691);        // 高度
        mount_roll_  = declare_parameter("mount_roll", 0.0);
        mount_pitch_ = declare_parameter("mount_pitch", 0.3839); // 22°

        // 订阅大yaw: 串口节点发了 /yaw
        yaw_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/yaw", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                publishTF(msg->data);
            });
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 兜底: 启动时先发一次 yaw=0,等电控实时覆盖
        publishTF(0.0);
    }

private:
    void publishTF(double yaw_angle) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->now();
        t.header.frame_id = "base_link";
        t.child_frame_id  = "livox_frame";

        t.transform.translation.x = mount_x_;
        t.transform.translation.y = mount_y_;
        t.transform.translation.z = mount_z_;

        tf2::Quaternion q;
        q.setRPY(mount_roll_, mount_pitch_, yaw_angle);  // RPY
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    double mount_x_, mount_y_, mount_z_, mount_roll_, mount_pitch_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}