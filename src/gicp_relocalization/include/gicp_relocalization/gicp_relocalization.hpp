#ifndef GICP_RELOCALIZATION_HPP_
#define GICP_RELOCALIZATION_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gicp_relocalization/global_search_2d.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace gicp_relocalization
{

enum class RelocState
{
  WAIT_INIT,  // C++ 自动全局搜索（同时接受 RViz 手动 /initialpose）
  TRACKING,   // GICP 连续跟踪
  LOST        // GICP 连续失败, 请求重新全局定位并等待
};

class GicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit GicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  void loadGlobalMap(const std::string & file_name);
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void runAutomaticInitialization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, uint64_t init_epoch);
  bool validateCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<SearchCandidate> & candidates,
    Eigen::Isometry3d & pose_out);
  void performRegistration();
  void publishTransform();
  bool prepareSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud);
  bool alignPreparedSource(
    const Eigen::Isometry3d & initial_guess,
    Eigen::Isometry3d & pose_out,
    double & inlier_ratio_out,
    double & rmse_out);
  bool tryGicpAlignment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
    const Eigen::Isometry3d & initial_guess,
    Eigen::Isometry3d & pose_out,
    double & inlier_ratio_out,
    double & rmse_out);
  Eigen::Isometry3d projectToPlanar(const Eigen::Isometry3d & pose) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;

  // 配准(全局搜索可阻塞 ~8s)单独一个回调组, 由 MultiThreadedExecutor 与
  // 点云订阅 / map->odom 发布并行执行, 搜索期间 TF 不断。
  rclcpp::CallbackGroup::SharedPtr registration_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  // 保护跨回调组共享的状态: 累积点云、reloc_state_、result_t_ 等。
  // 只在读写时短暂持有, GICP / 全局搜索在锁外进行。
  std::mutex state_mtx_;
  // 每次 /initialpose 递增; 锁外配准完成后若已变化, 说明期间收到手动初值, 丢弃本次结果
  uint64_t init_epoch_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  RelocState reloc_state_ = RelocState::WAIT_INIT;

  double global_leaf_size_;
  double registered_leaf_size_;
  int num_threads_;
  int num_neighbors_;
  double max_dist_sq_;
  int gicp_max_iterations_;
  int gicp_max_consecutive_failures_;
  int gicp_fail_count_ = 0;
  double global_min_accum_sec_;
  double global_retry_period_sec_;
  double validation_max_dist_;
  int validation_max_points_;
  double candidate_min_inlier_ratio_;
  double candidate_max_rmse_;
  double candidate_ambiguity_ratio_;
  double candidate_max_correction_translation_;
  double candidate_max_correction_yaw_;
  double candidate_dedup_distance_;
  double candidate_dedup_yaw_deg_;
  double tracking_min_inlier_ratio_;
  double tracking_max_rmse_;
  double tracking_max_jump_translation_;
  double tracking_max_jump_yaw_;
  bool force_planar_;
  // false 时禁用自动全局搜索,只在 WAIT_INIT 状态下等 RViz /initialpose。
  // 用于地图/环境对不上时,避免 CPU 被搜索占满,改由人工给初始位姿
  bool auto_global_search_ = true;
  GlobalSearchConfig global_search_config_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  std::string prior_pcd_file_;
  std::string input_cloud_topic_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time first_accumulation_time_;
  rclcpp::Time last_global_search_time_;

  Eigen::Isometry3d result_t_;
  // 曾经成功定位过: 之后即使跟丢/重新全局搜索, 也继续发布最近一次的 map->odom
  bool has_result_ = false;
  Eigen::Isometry3d previous_result_t_;

  // 智能跟踪:记录上次成功跟踪时的 odom 位姿,用于判断是否需要重新配准
  Eigen::Vector3d last_track_odom_pos_{0, 0, 0};  // 上次跟踪时 base_link 在 odom 系的位置
  rclcpp::Time last_successful_track_time_;        // 上次成功跟踪的时间
  bool last_track_odom_valid_ = false;             // 是否已初始化 last_track_odom_pos_

  // 智能跟踪参数(在 performRegistration 里直接用)
  static constexpr double TRACK_MIN_TRANSLATION = 0.3;   // 位移超过 30cm 才配准
  static constexpr double TRACK_MIN_YAW_DEG = 5.0;       // 旋转超过 5° 才配准
  static constexpr double TRACK_FORCE_INTERVAL_SEC = 10.0; // 最长 10 秒强制配准一次

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;
  std::unique_ptr<GlobalSearch2D> global_search_;
};

}  // namespace gicp_relocalization

#endif  // GICP_RELOCALIZATION_HPP_
