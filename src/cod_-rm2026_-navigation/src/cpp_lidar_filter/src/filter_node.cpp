#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/time.h>                          // ← durationFromSec / TimePointZero

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode() : Node("lidar_filter_node")
  {
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/livox/lidar_filtered");
    // 裁剪参数 (CropBox)
    this->declare_parameter("min_x", -0.26);
    this->declare_parameter("max_x",  0.26);
    this->declare_parameter("min_y", -0.26);
    this->declare_parameter("max_y",  0.26);
    this->declare_parameter("min_z", -0.05);
    this->declare_parameter("max_z",  0.60);
    this->declare_parameter("negative", true); // true = 挖掉中间
    // 降采样参数
    this->declare_parameter("leaf_size", 0.05);

    // --- 新增: 帧参数（默认 base_link 挖盒, livox 输出）---
    this->declare_parameter("target_frame", "base_link");  // 在哪系挖车身
    this->declare_parameter("livox_frame", "livox_frame"); // 输入点云所在系

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());

    // --- 新增: TF 缓冲/监听 ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void publish_marker()
  {
    double min_x = this->get_parameter("min_x").as_double();
    double max_x = this->get_parameter("max_x").as_double();
    double min_y = this->get_parameter("min_y").as_double();
    double max_y = this->get_parameter("max_y").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double max_z = this->get_parameter("max_z").as_double();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->get_parameter("target_frame").as_string();
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 计算中心点位置
    marker.pose.position.x = (max_x + min_x) / 2.0;
    marker.pose.position.y = (max_y + min_y) / 2.0;
    marker.pose.position.z = (max_z + min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    // 计算尺寸
    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = max_z - min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4; // 半透明

    marker_pub_->publish(marker);
  }
     void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::string livox_frame  = this->get_parameter("livox_frame").as_string();
    std::string target_frame = this->get_parameter("target_frame").as_string();

    // ==== ① 取 livox → target(base_link) 变换 ====
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame, livox_frame,
        tf2::TimePointZero, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s->%s 不可用: %s", target_frame.c_str(), livox_frame.c_str(), ex.what());
      return;
    }

    // ==== ② 点云旋到 base_link ====
    sensor_msgs::msg::PointCloud2 cloud_tgt_msg;
    tf2::doTransform(*msg, cloud_tgt_msg, tf);
    pcl::PCLPointCloud2::Ptr cloud_in_tgt(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud_tgt_msg, *cloud_in_tgt);

    // ==== ③ 在 base_link 系 CropBox 挖车身 ====
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(cloud_in_tgt);
    Eigen::Vector4f min_pt, max_pt;
    min_pt << this->get_parameter("min_x").as_double(),
              this->get_parameter("min_y").as_double(),
              this->get_parameter("min_z").as_double(), 1.0f;
    max_pt << this->get_parameter("max_x").as_double(),
              this->get_parameter("max_y").as_double(),
              this->get_parameter("max_z").as_double(), 1.0f;
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(this->get_parameter("negative").as_bool());
    pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);
    crop.filter(*cloud_cropped);

    // ==== ④ 直接输出 base_link 系（不再回旋 livox！）====
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_cropped, output);
    output.header = msg->header;
    output.header.frame_id = target_frame;   // = "base_link"
    pub_->publish(output);
  }
  // ==== 新增成员 ====
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}