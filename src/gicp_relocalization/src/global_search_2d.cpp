#include "gicp_relocalization/global_search_2d.hpp"

#include <unsupported/Eigen/FFT>

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace gicp_relocalization
{

namespace
{
using Complex = std::complex<float>;

void fft2(
  Eigen::FFT<float> & fft,
  const std::vector<Complex> & input,
  std::vector<Complex> & output,
  int rows, int cols, bool inverse)
{
  std::vector<Complex> row_transformed(input.size());
  for (int row = 0; row < rows; ++row) {
    const Complex * source = input.data() + static_cast<size_t>(row) * cols;
    Complex * target = row_transformed.data() + static_cast<size_t>(row) * cols;
    if (inverse) {
      fft.inv(target, source, cols);
    } else {
      fft.fwd(target, source, cols);
    }
  }

  output.resize(input.size());
  std::vector<Complex> column_in(rows);
  std::vector<Complex> column_out(rows);
  for (int col = 0; col < cols; ++col) {
    for (int row = 0; row < rows; ++row) {
      column_in[row] = row_transformed[static_cast<size_t>(row) * cols + col];
    }
    if (inverse) {
      fft.inv(column_out.data(), column_in.data(), rows);
    } else {
      fft.fwd(column_out.data(), column_in.data(), rows);
    }
    for (int row = 0; row < rows; ++row) {
      output[static_cast<size_t>(row) * cols + col] = column_out[row];
    }
  }
}

int findLayer(const std::vector<double> & layers, double z)
{
  for (size_t i = 0; i + 1 < layers.size(); ++i) {
    if (z > layers[i] && z <= layers[i + 1]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}
}  // namespace

GlobalSearch2D::GlobalSearch2D(
  const pcl::PointCloud<pcl::PointXYZ> & map,
  const GlobalSearchConfig & config)
: config_(config)
{
  if (
    config_.z_layers.size() < 2 || config_.resolution <= 0.0 ||
    config_.yaw_step_deg <= 0.0)
  {
    throw std::invalid_argument("Invalid global-search configuration");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : map) {
    if (findLayer(config_.z_layers, point.z) < 0) {
      continue;
    }
    min_x = std::min<double>(min_x, point.x);
    min_y = std::min<double>(min_y, point.y);
    max_x = std::max<double>(max_x, point.x);
    max_y = std::max<double>(max_y, point.y);
  }
  if (!std::isfinite(min_x)) {
    throw std::runtime_error("Prior map has no points in configured z layers");
  }

  constexpr int kMarginCells = 10;
  map_origin_x_ =
    std::floor(min_x / config_.resolution) * config_.resolution -
    kMarginCells * config_.resolution;
  map_origin_y_ =
    std::floor(min_y / config_.resolution) * config_.resolution -
    kMarginCells * config_.resolution;
  map_rows_ = static_cast<int>(
    std::ceil((max_x - map_origin_x_) / config_.resolution)) + kMarginCells + 1;
  map_cols_ = static_cast<int>(
    std::ceil((max_y - map_origin_y_) / config_.resolution)) + kMarginCells + 1;

  map_layers_.resize(config_.z_layers.size() - 1);
  for (auto & layer : map_layers_) {
    layer.rows = map_rows_;
    layer.cols = map_cols_;
    layer.data.assign(map_rows_ * map_cols_, 0.0f);
  }

  for (const auto & point : map) {
    const int layer = findLayer(config_.z_layers, point.z);
    if (layer < 0) {
      continue;
    }
    const int row = static_cast<int>(
      std::floor((point.x - map_origin_x_) / config_.resolution));
    const int col = static_cast<int>(
      std::floor((point.y - map_origin_y_) / config_.resolution));
    if (row >= 0 && row < map_rows_ && col >= 0 && col < map_cols_) {
      map_layers_[layer].at(row, col) = 1.0f;
    }
  }
  for (auto & layer : map_layers_) {
    dilateOneCell(layer);
  }
}

std::vector<SearchCandidate> GlobalSearch2D::search(
  const pcl::PointCloud<pcl::PointXYZ> & source,
  GlobalSearchStats * stats) const
{
  std::vector<std::unordered_set<std::uint64_t>> cell_sets(map_layers_.size());
  for (const auto & point : source) {
    const int layer = findLayer(config_.z_layers, point.z);
    if (layer < 0) {
      continue;
    }
    const int x = static_cast<int>(std::llround(point.x / config_.resolution));
    const int y = static_cast<int>(std::llround(point.y / config_.resolution));
    cell_sets[layer].insert(cellKey(x, y));
  }

  int occupied_count = 0;
  double radius = config_.resolution;
  std::vector<std::vector<std::pair<double, double>>> layer_points(cell_sets.size());
  for (size_t layer = 0; layer < cell_sets.size(); ++layer) {
    layer_points[layer].reserve(cell_sets[layer].size());
    for (const auto key : cell_sets[layer]) {
      const auto x = static_cast<std::int32_t>(key >> 32);
      const auto y = static_cast<std::int32_t>(key & 0xFFFFFFFFu);
      const double px = static_cast<double>(x) * config_.resolution;
      const double py = static_cast<double>(y) * config_.resolution;
      layer_points[layer].emplace_back(px, py);
      radius = std::max(radius, std::max(std::abs(px), std::abs(py)));
    }
    occupied_count += static_cast<int>(layer_points[layer].size());
  }
  if (stats) {
    stats->occupied_cells = occupied_count;
    stats->best_score = 0.0f;
  }
  if (occupied_count < config_.min_occupied_cells) {
    return {};
  }

  radius = std::ceil(radius / config_.resolution) * config_.resolution +
    config_.resolution;
  const double scan_origin = -radius;
  const int scan_side =
    static_cast<int>(std::ceil(2.0 * radius / config_.resolution)) + 1;
  const int correlation_rows = map_rows_ + scan_side - 1;
  const int correlation_cols = map_cols_ + scan_side - 1;
  const int fft_rows = nextPowerOfTwo(correlation_rows);
  const int fft_cols = nextPowerOfTwo(correlation_cols);
  const size_t fft_size = static_cast<size_t>(fft_rows) * fft_cols;

  Eigen::FFT<float> fft;
  std::vector<std::vector<Complex>> map_spectra(map_layers_.size());
  for (size_t layer = 0; layer < map_layers_.size(); ++layer) {
    std::vector<Complex> padded(fft_size, Complex(0.0f, 0.0f));
    for (int row = 0; row < map_rows_; ++row) {
      for (int col = 0; col < map_cols_; ++col) {
        padded[static_cast<size_t>(row) * fft_cols + col] =
          Complex(map_layers_[layer].at(row, col), 0.0f);
      }
    }
    fft2(fft, padded, map_spectra[layer], fft_rows, fft_cols, false);
  }

  const int suppression_radius = std::max(1, config_.peak_window_cells / 2);
  const double yaw_step = config_.yaw_step_deg * M_PI / 180.0;
  const int yaw_count = static_cast<int>(std::ceil(2.0 * M_PI / yaw_step));
  std::vector<std::vector<SearchCandidate>> candidates_by_yaw(yaw_count);

#pragma omp parallel for schedule(dynamic) num_threads(config_.num_threads)
  for (int yaw_index = 0; yaw_index < yaw_count; ++yaw_index) {
    const double yaw = yaw_index * yaw_step;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::FFT<float> yaw_fft;
    std::vector<Complex> spectrum_sum(fft_size, Complex(0.0f, 0.0f));
    int scan_cell_count = 0;

    for (size_t layer = 0; layer < layer_points.size(); ++layer) {
      if (layer_points[layer].empty()) {
        continue;
      }
      std::vector<Complex> reversed_scan(fft_size, Complex(0.0f, 0.0f));
      for (const auto & point : layer_points[layer]) {
        const double rx = c * point.first - s * point.second;
        const double ry = s * point.first + c * point.second;
        const int row = static_cast<int>(
          std::llround((rx - scan_origin) / config_.resolution));
        const int col = static_cast<int>(
          std::llround((ry - scan_origin) / config_.resolution));
        if (row < 0 || row >= scan_side || col < 0 || col >= scan_side) {
          continue;
        }
        reversed_scan[
          static_cast<size_t>(scan_side - 1 - row) * fft_cols +
          (scan_side - 1 - col)] = Complex(1.0f, 0.0f);
      }

      std::vector<Complex> scan_spectrum;
      fft2(yaw_fft, reversed_scan, scan_spectrum, fft_rows, fft_cols, false);
      for (size_t i = 0; i < fft_size; ++i) {
        spectrum_sum[i] += map_spectra[layer][i] * scan_spectrum[i];
      }
      scan_cell_count += static_cast<int>(layer_points[layer].size());
    }

    std::vector<Complex> correlation;
    fft2(yaw_fft, spectrum_sum, correlation, fft_rows, fft_cols, true);

    std::vector<std::pair<int, int>> selected_peaks;
    for (int peak = 0; peak < config_.peaks_per_yaw; ++peak) {
      float best_score = -std::numeric_limits<float>::infinity();
      int best_row = -1;
      int best_col = -1;
      for (int row = 0; row < correlation_rows; ++row) {
        for (int col = 0; col < correlation_cols; ++col) {
          bool suppressed = false;
          for (const auto & selected : selected_peaks) {
            if (
              std::abs(row - selected.first) <= suppression_radius &&
              std::abs(col - selected.second) <= suppression_radius)
            {
              suppressed = true;
              break;
            }
          }
          if (suppressed) {
            continue;
          }
          const float score =
            correlation[static_cast<size_t>(row) * fft_cols + col].real() /
            std::max(1, scan_cell_count);
          if (score > best_score) {
            best_score = score;
            best_row = row;
            best_col = col;
          }
        }
      }
      if (best_row < 0) {
        break;
      }
      selected_peaks.emplace_back(best_row, best_col);
      candidates_by_yaw[yaw_index].push_back(
        SearchCandidate{
          best_score,
          yaw,
          map_origin_x_ +
          (best_row - (scan_side - 1)) * config_.resolution - scan_origin,
          map_origin_y_ +
          (best_col - (scan_side - 1)) * config_.resolution - scan_origin});
    }
  }

  std::vector<SearchCandidate> raw_candidates;
  raw_candidates.reserve(yaw_count * config_.peaks_per_yaw);
  for (const auto & yaw_candidates : candidates_by_yaw) {
    raw_candidates.insert(
      raw_candidates.end(), yaw_candidates.begin(), yaw_candidates.end());
  }

  std::sort(
    raw_candidates.begin(), raw_candidates.end(),
    [](const SearchCandidate & lhs, const SearchCandidate & rhs) {
      return lhs.score > rhs.score;
    });
  if (stats && !raw_candidates.empty()) {
    stats->best_score = raw_candidates.front().score;
  }
  if (
    raw_candidates.empty() ||
    raw_candidates.front().score < config_.match_threshold)
  {
    return {};
  }

  std::vector<SearchCandidate> candidates;
  const double min_yaw =
    config_.candidate_min_yaw_deg * M_PI / 180.0;
  for (const auto & candidate : raw_candidates) {
    bool distinct = true;
    for (const auto & kept : candidates) {
      const double distance = std::hypot(candidate.x - kept.x, candidate.y - kept.y);
      if (
        distance < config_.candidate_min_translation &&
        normalizedYawDistance(candidate.yaw, kept.yaw) < min_yaw)
      {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      candidates.push_back(candidate);
    }
    if (static_cast<int>(candidates.size()) >= config_.top_k) {
      break;
    }
  }
  return candidates;
}

int GlobalSearch2D::nextPowerOfTwo(int value)
{
  int result = 1;
  while (result < value) {
    result <<= 1;
  }
  return result;
}

std::uint64_t GlobalSearch2D::cellKey(int x, int y)
{
  return
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
    static_cast<std::uint32_t>(y);
}

double GlobalSearch2D::normalizedYawDistance(double lhs, double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

void GlobalSearch2D::dilateOneCell(BinaryGrid & grid)
{
  const auto original = grid.data;
  for (int row = 0; row < grid.rows; ++row) {
    for (int col = 0; col < grid.cols; ++col) {
      if (original[static_cast<size_t>(row) * grid.cols + col] <= 0.0f) {
        continue;
      }
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          const int nr = row + dr;
          const int nc = col + dc;
          if (nr >= 0 && nr < grid.rows && nc >= 0 && nc < grid.cols) {
            grid.at(nr, nc) = 1.0f;
          }
        }
      }
    }
  }
}

}  // namespace gicp_relocalization
