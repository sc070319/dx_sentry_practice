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
        // 2026-09-16 实测修正: 倾斜方向实测为绕雷达 x 轴(roll), 不是绕 y 轴(pitch)
        // 依据: IMU 重力 21.4° 与点云地面法线 21.7° 完全一致, 都往 +y 倒
        mount_roll_  = declare_parameter("mount_roll", 0.3839);   // 22°
        mount_pitch_ = declare_parameter("mount_pitch", 0.0);

        // 订阅大yaw: 只更新缓存值, 由下方定频定时器统一发布
        yaw_subscription_ = create_subscription<std_msgs::msg::Float64>(
            "/yaw", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                last_yaw_ = msg->data;
            });
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 定频持续发布(默认100Hz)。
        // 原来只在收到 /yaw 时才发, 串口节点5ms批量读取造成 TF 有毫秒级空档,
        // LIO 按里程计时间戳查询会偶发 "extrapolation into the future" 并丢弃整帧 odom 输出
        const double publish_rate = declare_parameter("publish_rate", 100.0);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            [this]() { publishTF(last_yaw_); });
        publishTF(last_yaw_);   // 启动时立即先发一次
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
    double last_yaw_ = 0.0;                 // 最新一次 /yaw 值(供定频发布)
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}