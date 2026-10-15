// Kino A* -> MINCO -> MPPI/MPC 的 ROS 2 节点 (不依赖 Nav2)
//   输入: /map (OccupancyGrid, transient_local), /goal_pose, TF map->base_link
//   输出: /cmd_vel, /kino_path, /plan (MINCO), /local_plan (控制器预测)
#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "omni_motion_planner/map/esdf_map.hpp"
#include "omni_motion_planner/search/kino_astar.hpp"
#include "omni_motion_planner/tracking/mppi_controller.hpp"
#include "omni_motion_planner/tracking/trajectory_tracker.hpp"
#include "omni_motion_planner/trajectory/minco_optimizer.hpp"

namespace omni_planner
{

// 控制器按"索引 = dt 步"取参考点, 所以参考轨迹要按控制器 dt 等时间重采样
static std::vector<TrajectoryPoint> resampleByTime(const std::vector<TrajectoryPoint> &p, double dt)
{
    std::vector<TrajectoryPoint> out;
    std::size_t j = 0;
    for (double t = 0.0;; t += dt) {
        while (j + 1 < p.size() && p[j + 1].t < t) ++j;
        if (j + 1 >= p.size()) { out.push_back(p.back()); break; }
        double a = (t - p[j].t) / std::max(1e-9, p[j + 1].t - p[j].t);
        TrajectoryPoint q = p[j];
        q.pos = (1.0 - a) * p[j].pos + a * p[j + 1].pos;
        q.theta = p[j].theta + a * std::remainder(p[j + 1].theta - p[j].theta, 2.0 * M_PI);
        q.t = t;
        out.push_back(q);
    }
    return out;
}

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("omni_motion_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        global_frame_ = declare_parameter("global_frame", std::string("map"));
        robot_frame_  = declare_parameter("robot_frame", std::string("base_link"));
        goal_tol_     = declare_parameter("goal_tolerance", 0.15);
        replan_dist_  = declare_parameter("replan_distance", 0.8);

        param("kino.robot_radius", kino_p_.robot_radius);
        param("kino.max_vel", kino_p_.max_vel);
        param("kino.max_acc", kino_p_.max_acc);
        param("kino.max_omega", kino_p_.max_omega);
        param("kino.dt", kino_p_.dt);
        param("kino.n_acc", kino_p_.n_acc);
        param("kino.n_omega", kino_p_.n_omega);
        param("kino.goal_tol_xy", kino_p_.goal_tol_xy);
        param("kino.goal_tol_theta", kino_p_.goal_tol_theta);
        param("kino.lambda", kino_p_.lambda);
        param("kino.lambda_obs", kino_p_.lambda_obs);
        param("kino.obs_decay", kino_p_.obs_decay);
        param("kino.xy_resolution", kino_p_.xy_resolution);
        param("kino.h_weight", kino_p_.h_weight);
        kino_p_.max_nodes = declare_parameter("kino.max_nodes", 200000);

        param("minco.w_smooth", minco_p_.w_smooth);
        param("minco.w_collision", minco_p_.w_collision);
        param("minco.w_length", minco_p_.w_length);
        param("minco.safe_distance", minco_p_.safe_distance);
        param("minco.max_vel", minco_p_.max_vel);
        param("minco.max_iters", minco_p_.max_iters);
        param("minco.learning_rate", minco_p_.learning_rate);
        param("minco.converge_threshold", minco_p_.converge_threshold);

        const auto type = declare_parameter("controller", std::string("mppi"));
        if (type == "mpc") {
            MpcParams p;
            loadTrackingParams(p);
            param("mpc.max_iters", p.max_iters);
            param("mpc.learning_rate", p.learning_rate);
            ctrl_dt_ = p.dt;
            controller_ = std::make_unique<MPC>(p);
        } else {
            MppiParams p;
            loadTrackingParams(p);
            param("mppi.num_samples", p.num_samples);
            param("mppi.sigma_v", p.sigma_v);
            param("mppi.sigma_omega", p.sigma_omega);
            param("mppi.lambda", p.lambda);
            ctrl_dt_ = p.dt;
            controller_ = std::make_unique<MPPI>(p);
        }
        RCLCPP_INFO(get_logger(), "controller: %s, dt=%.3f", type == "mpc" ? "MPC" : "MPPI", ctrl_dt_);

        cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        kino_pub_  = create_publisher<nav_msgs::msg::Path>("kino_path", 1);
        plan_pub_  = create_publisher<nav_msgs::msg::Path>("plan", 1);
        local_pub_ = create_publisher<nav_msgs::msg::Path>("local_plan", 1);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { onMap(*m); });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr g) { onGoal(*g); });

        timer_ = create_timer(std::chrono::duration<double>(ctrl_dt_), [this] { controlLoop(); });
    }

private:
    template <class T>
    void param(const std::string &name, T &value) { value = declare_parameter(name, value); }

    // MPC / MPPI 共用的 horizon、限幅和代价权重
    template <class P>
    void loadTrackingParams(P &p)
    {
        param("tracking.horizon", p.horizon);
        param("tracking.dt", p.dt);
        param("tracking.max_vel", p.max_vel);
        param("tracking.max_omega", p.max_omega);
        param("tracking.max_acc", p.max_acc);
        param("tracking.max_alpha", p.max_alpha);
        param("tracking.w_pos", p.w_pos);
        param("tracking.w_theta", p.w_theta);
        param("tracking.w_vel", p.w_vel);
        param("tracking.w_delta_vel", p.w_delta_vel);
        param("tracking.w_delta_omega", p.w_delta_omega);
    }

    void onMap(const nav_msgs::msg::OccupancyGrid &m)
    {
        const auto &info = m.info;
        std::vector<uint8_t> occ(m.data.size());
        for (std::size_t i = 0; i < occ.size(); ++i)
            occ[i] = (m.data[i] < 0 || m.data[i] >= 50) ? 255 : 0;  // 未知当障碍

        esdf_ = std::make_unique<ESDFMap>(info.width, info.height, info.resolution,
                                          info.origin.position.x, info.origin.position.y);
        esdf_->buildFromOccupancyGrid(occ);
        kino_  = std::make_unique<KinoDynamicAStar>(esdf_.get(), kino_p_);
        minco_ = std::make_unique<MincoOptimizer>(esdf_.get(), minco_p_);
        RCLCPP_INFO(get_logger(), "map %ux%u @ %.2fm, ESDF ready", info.width, info.height, info.resolution);
    }

    void onGoal(const geometry_msgs::msg::PoseStamped &g)
    {
        goal_ = Eigen::Vector3d(g.pose.position.x, g.pose.position.y, tf2::getYaw(g.pose.orientation));
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f, %.2f)", goal_.x(), goal_.y(), goal_.z());
        plan();
    }

    bool robotPose(Eigen::Vector3d &pose)
    {
        try {
            auto tf = tf_buffer_.lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
            pose = {tf.transform.translation.x, tf.transform.translation.y, tf2::getYaw(tf.transform.rotation)};
            return true;
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF %s->%s: %s",
                                 global_frame_.c_str(), robot_frame_.c_str(), e.what());
            return false;
        }
    }

    void plan()
    {
        ref_.clear();
        Eigen::Vector3d s;
        if (!esdf_ || !has_goal_ || !robotPose(s)) {
            RCLCPP_WARN(get_logger(), "plan skipped: map/goal/TF not ready");
            return;
        }

        const double r = kino_p_.robot_radius;
        if (!esdf_->isSafe(s.x(), s.y(), r) || !esdf_->isSafe(goal_.x(), goal_.y(), r)) {
            RCLCPP_ERROR(get_logger(), "start (%.2fm) or goal (%.2fm) clearance < robot_radius %.2f",
                         esdf_->getDistance(s.x(), s.y()), esdf_->getDistance(goal_.x(), goal_.y()), r);
            has_goal_ = false;
            stop();
            return;
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!kino_->planWithHeading(s.head<2>(), s.z(), goal_.head<2>(), goal_.z())) {
            RCLCPP_ERROR(get_logger(), "Kino A* failed (%zu nodes)", kino_->expandedStates().size());
            has_goal_ = false;
            stop();
            return;
        }
        const auto t1 = std::chrono::steady_clock::now();
        if (!minco_->optimize(kino_->path())) {
            RCLCPP_ERROR(get_logger(), "MINCO failed");
            has_goal_ = false;
            stop();
            return;
        }
        const auto t2 = std::chrono::steady_clock::now();

        ref_ = resampleByTime(minco_->smoothPath(), ctrl_dt_);
        controller_->reset();
        kino_pub_->publish(toPath(kino_->path()));
        plan_pub_->publish(toPath(ref_));

        auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
        RCLCPP_INFO(get_logger(), "plan ok: kino %.1fms (%zu nodes, %zu pts), minco %.1fms (%zu iters), %.1fs traj",
                    ms(t0, t1), kino_->expandedStates().size(), kino_->path().size(),
                    ms(t1, t2), minco_->lossHistory().size(), ref_.back().t);
    }

    void controlLoop()
    {
        if (ref_.empty()) return;
        Eigen::Vector3d s;
        if (!robotPose(s)) { stop(); return; }

        if ((s.head<2>() - ref_.back().pos).norm() < goal_tol_) {
            RCLCPP_INFO(get_logger(), "goal reached");
            ref_.clear();
            has_goal_ = false;
            stop();
            return;
        }

        double min_d = INFINITY;
        for (const auto &p : ref_) min_d = std::min(min_d, (p.pos - s.head<2>()).norm());
        if (min_d > replan_dist_) {
            RCLCPP_WARN(get_logger(), "off track %.2fm, replanning", min_d);
            stop();
            plan();
            return;
        }

        auto cmd = controller_->computeCommand(s, ref_);
        if (!cmd) { stop(); return; }
        geometry_msgs::msg::Twist tw;
        tw.linear.x  = cmd->v;
        tw.angular.z = cmd->omega;
        cmd_pub_->publish(tw);
        local_pub_->publish(toPath(controller_->predictedTrajectory()));
    }

    void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

    nav_msgs::msg::Path toPath(const std::vector<TrajectoryPoint> &pts) const
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = now();
        for (const auto &p : pts) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = p.pos.x();
            ps.pose.position.y = p.pos.y();
            ps.pose.orientation.z = std::sin(0.5 * p.theta);
            ps.pose.orientation.w = std::cos(0.5 * p.theta);
            path.poses.push_back(ps);
        }
        return path;
    }

    std::string global_frame_, robot_frame_;
    double goal_tol_, replan_dist_, ctrl_dt_{0.05};

    KinoParams kino_p_;
    MincoParams minco_p_;
    std::unique_ptr<ESDFMap> esdf_;
    std::unique_ptr<KinoDynamicAStar> kino_;
    std::unique_ptr<MincoOptimizer> minco_;
    std::unique_ptr<Controller> controller_;

    Eigen::Vector3d goal_{0.0, 0.0, 0.0};
    bool has_goal_{false};
    std::vector<TrajectoryPoint> ref_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr kino_pub_, plan_pub_, local_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni_planner

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<omni_planner::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
