// lio_odom_adapter: 把 Point-LIO 的输出接成 Nav2 需要的标准 TF / 话题
//
// Point-LIO 原生输出: camera_init(=初始 IMU 系, 已重力对齐) -> body(IMU) 的位姿, 没有速度.
// 本节点定义 odom = 启动时刻的 base_link (贴地、水平), 并发布:
//   TF  odom -> camera_init   (静态, 启动时确定)
//   TF  odom -> base_link     (随 Point-LIO 位姿更新)
//   /odom                     nav_msgs/Odometry, 带 body 系速度 (位姿差分 + 低通)
//   /lio/cloud_registered     cloud_registered 转到 odom 系 (给重定位 / 可视化)
// 建图模式 (accumulate_map=true) 下累积 odom 系体素地图, 调 /save_map 或退出时存 PCD.
//
//   T_odom_base(t) = T_base_imu · T_ci_imu(t0)^-1 · T_ci_imu(t) · T_imu_base

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <mutex>
#include <optional>

class LioOdomAdapter : public rclcpp::Node
{
public:
  LioOdomAdapter()
  : Node("lio_odom_adapter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    tf_pub_(this), static_pub_(this)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    lio_frame_ = declare_parameter<std::string>("lio_frame", "camera_init");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    vel_alpha_ = declare_parameter<double>("velocity_lpf_alpha", 0.3);
    accumulate_map_ = declare_parameter<bool>("accumulate_map", false);
    map_leaf_ = declare_parameter<double>("map_voxel_size", 0.05);
    map_path_ = declare_parameter<std::string>("map_save_path", "");

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lio/cloud_registered", 5);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("lio_odom_topic", "/aft_mapped_to_init"), 50,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdom(m);});
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("lio_cloud_topic", "/cloud_registered"), 5,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onCloud(m);});
    srv_save_ = create_service<std_srvs::srv::Trigger>("/save_map",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        res->success = saveMap(res->message);
      });
    map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  ~LioOdomAdapter() override
  {
    if (accumulate_map_ && !map_path_.empty()) {
      std::string msg;
      saveMap(msg);
    }
  }

private:
  bool ready()
  {
    if (T_base_imu_) {return true;}
    try {
      auto tf = tf_buffer_.lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
      T_base_imu_ = tf2::transformToEigen(tf);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting TF %s <- %s",
        base_frame_.c_str(), imu_frame_.c_str());
      return false;
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & m)
  {
    if (!ready()) {return;}
    Eigen::Isometry3d T_ci_imu;
    tf2::fromMsg(m->pose.pose, T_ci_imu);
    if (!T_odom_ci_) {
      T_odom_ci_ = (*T_base_imu_) * T_ci_imu.inverse();
      geometry_msgs::msg::TransformStamped st = tf2::eigenToTransform(*T_odom_ci_);
      st.header.stamp = m->header.stamp;
      st.header.frame_id = odom_frame_;
      st.child_frame_id = lio_frame_;
      static_pub_.sendTransform(st);
      RCLCPP_INFO(get_logger(), "odom frame initialised at first Point-LIO pose");
    }
    const Eigen::Isometry3d T = (*T_odom_ci_) * T_ci_imu * T_base_imu_->inverse();
    const rclcpp::Time stamp(m->header.stamp);

    nav_msgs::msg::Odometry o;
    o.header.stamp = m->header.stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose = tf2::toMsg(T);
    if (last_T_ && (stamp - last_stamp_).seconds() > 1e-4) {
      const double dt = (stamp - last_stamp_).seconds();
      const Eigen::Isometry3d d = last_T_->inverse() * T;   // body 系增量
      const Eigen::Vector3d v = d.translation() / dt;
      const Eigen::AngleAxisd aa(d.rotation());
      const Eigen::Vector3d w = aa.axis() * aa.angle() / dt;
      vel_ = vel_alpha_ * v + (1 - vel_alpha_) * vel_;
      omega_ = vel_alpha_ * w + (1 - vel_alpha_) * omega_;
    }
    o.twist.twist.linear.x = vel_.x();
    o.twist.twist.linear.y = vel_.y();
    o.twist.twist.linear.z = vel_.z();
    o.twist.twist.angular.x = omega_.x();
    o.twist.twist.angular.y = omega_.y();
    o.twist.twist.angular.z = omega_.z();
    pub_odom_->publish(o);
    last_T_ = T;
    last_stamp_ = stamp;

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t = tf2::eigenToTransform(T);
      t.header.stamp = m->header.stamp;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      tf_pub_.sendTransform(t);
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m)
  {
    if (!T_odom_ci_) {return;}
    pcl::PointCloud<pcl::PointXYZI> in, out;
    pcl::fromROSMsg(*m, in);
    pcl::transformPointCloud(in, out, T_odom_ci_->matrix().cast<float>());
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header.stamp = m->header.stamp;
    msg.header.frame_id = odom_frame_;
    pub_cloud_->publish(msg);

    if (accumulate_map_) {
      std::lock_guard<std::mutex> lk(map_mtx_);
      *map_ += out;
      if (map_->size() > last_filtered_size_ * 2 + 200000) {
        downsample();
      }
    }
  }

  void downsample()
  {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(map_leaf_, map_leaf_, map_leaf_);
    vg.setInputCloud(map_);
    auto f = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    vg.filter(*f);
    map_ = f;
    last_filtered_size_ = map_->size();
  }

  bool saveMap(std::string & message)
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    if (!accumulate_map_ || map_->empty() || map_path_.empty()) {
      message = "map accumulation disabled, empty, or map_save_path not set";
      return false;
    }
    downsample();
    pcl::io::savePCDFileBinary(map_path_, *map_);
    message = "saved " + std::to_string(map_->size()) + " points to " + map_path_;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  std::string odom_frame_, base_frame_, imu_frame_, lio_frame_, map_path_;
  bool publish_tf_, accumulate_map_;
  double vel_alpha_, map_leaf_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_pub_;
  tf2_ros::StaticTransformBroadcaster static_pub_;
  std::optional<Eigen::Isometry3d> T_base_imu_, T_odom_ci_, last_T_;
  rclcpp::Time last_stamp_;
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()}, omega_{Eigen::Vector3d::Zero()};

  std::mutex map_mtx_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
  size_t last_filtered_size_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LioOdomAdapter>();
  rclcpp::spin(node);
  node.reset();   // 析构时自动存图
  rclcpp::shutdown();
  return 0;
}
