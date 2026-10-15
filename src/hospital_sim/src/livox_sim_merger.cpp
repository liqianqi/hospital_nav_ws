// livox_sim_merger: 把 Gazebo 两台 gpu_lidar 的高频子帧合成 Mid360 风格的 10Hz Livox CustomMsg
//
// 为什么不直接用 gz 点云:
//   gz gpu_lidar 每帧是同一时刻的"快照", 没有运动畸变, 也没有逐点时间戳;
//   Point-LIO 逐点做状态更新, 需要逐点时间。这里让 gz 以 50Hz 出子帧,
//   每 100ms 把 5 个子帧 x 2 台雷达拼成一帧, 每个点的 offset_time = 子帧时刻 - 帧起点,
//   于是运动畸变和时间戳是自洽的 (粒度 20ms), 这更接近真实 Mid360 的非重复扫描。
//   每个子帧随机抽样 points_per_subscan 个点 → 模拟非重复扫描的不同覆盖。
//
// 输出 (都在主雷达 livox_frame 下):
//   /livox/lidar   livox_ros_driver2/CustomMsg   给 Point-LIO
//   /livox/points  sensor_msgs/PointCloud2       调试 / Nav2 costmap
//
// 第二台雷达的点通过 TF(livox_frame <- livox_frame_2) 变换到主雷达系, 与真机
// "双雷达合并后以主雷达 + 其内置 IMU 跑 LIO" 的做法一致。

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <cmath>
#include <deque>
#include <random>
#include <vector>

namespace
{
struct SubScan
{
  int64_t stamp_ns;
  std::vector<Eigen::Vector3f> pts;   // 已变换到主雷达系、已过滤
  std::vector<uint8_t> rings;
  std::vector<float> intensity;
};
}  // namespace

class LivoxSimMerger : public rclcpp::Node
{
public:
  LivoxSimMerger()
  : Node("livox_sim_merger"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), rng_(42)
  {
    main_frame_ = declare_parameter<std::string>("main_frame", "livox_frame");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto topics = declare_parameter<std::vector<std::string>>(
      "input_topics", {"/livox/front_right_raw", "/livox/back_left_raw"});
    frame_period_ns_ = static_cast<int64_t>(declare_parameter<double>("frame_period", 0.1) * 1e9);
    points_per_subscan_ = declare_parameter<int>("points_per_subscan", 4000);
    scan_lines_ = declare_parameter<int>("scan_lines", 4);
    min_range_ = declare_parameter<double>("min_range", 0.1);
    self_box_ = declare_parameter<std::vector<double>>(
      "self_filter_box", {-0.30, 0.30, -0.30, 0.30, -0.10, 0.60});  // base_link 下 xmin xmax ymin ymax zmin zmax
    max_wait_ns_ = static_cast<int64_t>(declare_parameter<double>("max_wait", 0.3) * 1e9);

    pub_livox_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10);
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points", 10);

    queues_.resize(topics.size());
    for (size_t i = 0; i < topics.size(); ++i) {
      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topics[i], rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(i, msg);}));
    }
    RCLCPP_INFO(get_logger(), "merging %zu lidars into %s, frame %.0f ms",
      topics.size(), main_frame_.c_str(), frame_period_ns_ * 1e-6);
  }

private:
  bool lookup(const std::string & target, const std::string & source, Eigen::Isometry3f & T)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(target, source, tf2::TimePointZero);
      T = tf2::transformToEigen(tf).cast<float>();
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting TF %s <- %s: %s",
        target.c_str(), source.c_str(), e.what());
      return false;
    }
  }

  void onCloud(size_t idx, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    const std::string & src = msg->header.frame_id;
    auto it = to_main_.find(src);
    if (it == to_main_.end()) {
      Eigen::Isometry3f T_main_src, T_base_src;
      if (!lookup(main_frame_, src, T_main_src) || !lookup(base_frame_, src, T_base_src)) {
        return;
      }
      it = to_main_.emplace(src, std::make_pair(T_main_src, T_base_src)).first;
    }
    const auto & [T_main_src, T_base_src] = it->second;

    // 读 x,y,z,(intensity),(ring). gz 的 PointCloudPacked 带 intensity 和 ring
    bool has_int = false, has_ring = false;
    for (const auto & f : msg->fields) {
      has_int |= f.name == "intensity";
      has_ring |= f.name == "ring";
    }
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    std::vector<uint32_t> valid;
    valid.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    std::vector<Eigen::Vector3f> raw(n);
    for (size_t k = 0; k < n; ++k, ++ix, ++iy, ++iz) {
      raw[k] = {*ix, *iy, *iz};
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {continue;}
      if (raw[k].squaredNorm() < min_range_ * min_range_) {continue;}
      const Eigen::Vector3f pb = T_base_src * raw[k];
      if (pb.x() > self_box_[0] && pb.x() < self_box_[1] && pb.y() > self_box_[2] &&
        pb.y() < self_box_[3] && pb.z() > self_box_[4] && pb.z() < self_box_[5])
      {
        continue;  // 车体自遮挡
      }
      valid.push_back(static_cast<uint32_t>(k));
    }
    // 随机抽样模拟非重复扫描
    if (points_per_subscan_ > 0 && static_cast<int>(valid.size()) > points_per_subscan_) {
      std::shuffle(valid.begin(), valid.end(), rng_);
      valid.resize(points_per_subscan_);
    }

    SubScan s;
    s.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    s.pts.reserve(valid.size());
    std::vector<float> inten(n, 100.f);
    std::vector<uint16_t> ring(n, 0);
    if (has_int) {
      sensor_msgs::PointCloud2ConstIterator<float> ii(*msg, "intensity");
      for (size_t k = 0; k < n; ++k, ++ii) {inten[k] = *ii;}
    }
    if (has_ring) {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> ir(*msg, "ring");
      for (size_t k = 0; k < n; ++k, ++ir) {ring[k] = *ir;}
    }
    for (auto k : valid) {
      s.pts.push_back(T_main_src * raw[k]);
      s.rings.push_back(static_cast<uint8_t>(ring[k] % scan_lines_));
      s.intensity.push_back(std::clamp(inten[k], 0.f, 255.f));
    }
    queues_[idx].push_back(std::move(s));
    tryEmit();
  }

  void tryEmit()
  {
    if (frame_start_ns_ < 0) {
      for (auto & q : queues_) {
        if (!q.empty()) {
          frame_start_ns_ = frame_start_ns_ < 0 ? q.front().stamp_ns :
            std::min(frame_start_ns_, q.front().stamp_ns);
        }
      }
      if (frame_start_ns_ < 0) {return;}
    }
    while (true) {
      const int64_t end = frame_start_ns_ + frame_period_ns_;
      int64_t newest = -1;
      bool all_done = true;
      for (auto & q : queues_) {
        const int64_t last = q.empty() ? -1 : q.back().stamp_ns;
        newest = std::max(newest, last);
        all_done &= last >= end;
      }
      // 所有雷达都已越过窗口末尾才出帧; 某台雷达掉线超过 max_wait 则只用在线的
      if (!all_done && newest < end + max_wait_ns_) {return;}
      emit(frame_start_ns_, end);
      frame_start_ns_ = end;
    }
  }

  void emit(int64_t start, int64_t end)
  {
    livox_ros_driver2::msg::CustomMsg out;
    out.header.stamp = rclcpp::Time(start, RCL_ROS_TIME);
    out.header.frame_id = main_frame_;
    out.timebase = static_cast<uint64_t>(start);
    out.lidar_id = 0;

    for (auto & q : queues_) {
      while (!q.empty() && q.front().stamp_ns < end) {
        const SubScan & s = q.front();
        if (s.stamp_ns >= start) {
          const uint32_t off = static_cast<uint32_t>(s.stamp_ns - start);
          for (size_t k = 0; k < s.pts.size(); ++k) {
            livox_ros_driver2::msg::CustomPoint p;
            p.offset_time = off;
            p.x = s.pts[k].x();
            p.y = s.pts[k].y();
            p.z = s.pts[k].z();
            p.reflectivity = static_cast<uint8_t>(s.intensity[k]);
            p.tag = 0x10;
            p.line = s.rings[k];
            out.points.push_back(p);
          }
        }
        q.pop_front();
      }
    }
    // Point-LIO 期望按时间排序
    std::stable_sort(out.points.begin(), out.points.end(),
      [](const auto & a, const auto & b) {return a.offset_time < b.offset_time;});
    out.point_num = static_cast<uint32_t>(out.points.size());
    if (out.points.empty()) {return;}

    if (pub_cloud_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 pc;
      pc.header = out.header;
      sensor_msgs::PointCloud2Modifier mod(pc);
      mod.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "t", 1, sensor_msgs::msg::PointField::FLOAT32);  // 相对帧起点, 秒
      mod.resize(out.points.size());
      sensor_msgs::PointCloud2Iterator<float> x(pc, "x"), y(pc, "y"), z(pc, "z"),
      in(pc, "intensity"), t(pc, "t");
      for (const auto & p : out.points) {
        *x = p.x; *y = p.y; *z = p.z; *in = p.reflectivity; *t = p.offset_time * 1e-9f;
        ++x; ++y; ++z; ++in; ++t;
      }
      pub_cloud_->publish(pc);
    }
    pub_livox_->publish(out);
  }

  std::string main_frame_, base_frame_;
  int64_t frame_period_ns_, max_wait_ns_;
  int points_per_subscan_, scan_lines_;
  double min_range_;
  std::vector<double> self_box_;
  int64_t frame_start_ns_{-1};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, std::pair<Eigen::Isometry3f, Eigen::Isometry3f>> to_main_;
  std::vector<std::deque<SubScan>> queues_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_livox_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxSimMerger>());
  rclcpp::shutdown();
  return 0;
}
