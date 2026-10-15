#ifndef GICP_RELOCALIZATION_GLOBAL_SEARCH_2D_HPP_
#define GICP_RELOCALIZATION_GLOBAL_SEARCH_2D_HPP_

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gicp_relocalization
{

struct SearchCandidate
{
  double score;
  double yaw;
  double x;
  double y;
};

struct GlobalSearchConfig
{
  std::vector<double> z_layers{0.3, 0.8, 1.3, 1.8, 2.3};
  double resolution = 0.1;
  double yaw_step_deg = 2.0;
  double match_threshold = 0.85;
  int top_k = 12;
  int peaks_per_yaw = 3;
  int peak_window_cells = 11;
  double candidate_min_translation = 0.7;
  double candidate_min_yaw_deg = 8.0;
  int min_occupied_cells = 1200;
  int num_threads = 4;
};

/// 一次搜索的统计, 用于搜索失败时诊断 (分数离阈值多远 / 点是否太少)
struct GlobalSearchStats
{
  int occupied_cells = 0;   // 扫描在各层占据的格子数
  float best_score = 0.0f;  // 最佳候选分数 (扫描格落在地图占据格上的比例)
};

/// 多高度层 2D FFT 全局搜索。输入点云在 odom，输出候选直接是 T_map_odom。
class GlobalSearch2D
{
public:
  GlobalSearch2D(
    const pcl::PointCloud<pcl::PointXYZ> & map,
    const GlobalSearchConfig & config);

  std::vector<SearchCandidate> search(
    const pcl::PointCloud<pcl::PointXYZ> & source,
    GlobalSearchStats * stats = nullptr) const;

private:
  struct BinaryGrid
  {
    int rows = 0;
    int cols = 0;
    std::vector<float> data;

    float & at(int row, int col) {return data[row * cols + col];}
    const float & at(int row, int col) const {return data[row * cols + col];}
  };

  static int nextPowerOfTwo(int value);
  static std::uint64_t cellKey(int x, int y);
  static double normalizedYawDistance(double lhs, double rhs);
  static void dilateOneCell(BinaryGrid & grid);

  GlobalSearchConfig config_;
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  int map_rows_ = 0;
  int map_cols_ = 0;
  std::vector<BinaryGrid> map_layers_;
};

}  // namespace gicp_relocalization

#endif  // GICP_RELOCALIZATION_GLOBAL_SEARCH_2D_HPP_
