#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace classify_perception
{
struct TopFace
{
  cv::Vec3d center;
  double top_z;
  double width;
  double depth;
  double plane_error;
  double confidence;
  double yaw;
};

inline double quantile(std::vector<double> values, double fraction)
{
  const auto index = static_cast<std::size_t>(fraction * (values.size() - 1));
  auto at = values.begin() + index;
  std::nth_element(values.begin(), at, values.end());
  return *at;
}

// The workpieces are known 50 mm cubes resting on a horizontal workbench.
// Known height/size constrain geometry; x/y are estimated from observations.
// A side face or a cropped top patch cannot establish a trustworthy centre.
inline std::optional<TopFace> fit_top_face(const std::vector<cv::Vec3d>& points,
                                         double expected_top = 0.053,
                                         double cube_size = 0.05)
{
  std::vector<cv::Vec3d> patch;
  for (const auto& p : points) {
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
        std::abs(p[2] - expected_top) < 0.007) patch.push_back(p);
  }
  if (patch.size() < 150) return std::nullopt;

  cv::Vec3d mean(0, 0, 0);
  for (const auto& p : patch) mean += p;
  mean *= 1.0 / patch.size();
  cv::Matx22d covariance = cv::Matx22d::zeros();
  cv::Vec2d rhs(0, 0), slope;
  for (const auto& p : patch) {
    const auto d = p - mean;
    covariance(0, 0) += d[0] * d[0];
    covariance(0, 1) += d[0] * d[1];
    covariance(1, 0) += d[0] * d[1];
    covariance(1, 1) += d[1] * d[1];
    rhs[0] += d[0] * d[2]; rhs[1] += d[1] * d[2];
  }
  if (!cv::solve(covariance, rhs, slope, cv::DECOMP_CHOLESKY) ||
      std::hypot(slope[0], slope[1]) > std::tan(8.0 * CV_PI / 180.0)) return std::nullopt;

  std::vector<double> xs, ys, zs;
  double squared_error = 0;
  std::vector<cv::Vec3d> inliers;
  for (const auto& p : patch) {
    const double residual = p[2] - mean[2] - slope[0] * (p[0] - mean[0]) -
                            slope[1] * (p[1] - mean[1]);
    if (std::abs(residual) > 0.003) continue;
    inliers.push_back(p);
    xs.push_back(p[0]); ys.push_back(p[1]); zs.push_back(p[2]);
    squared_error += residual * residual;
  }
  if (inliers.size() < 150 || inliers.size() < patch.size() * 0.8) return std::nullopt;
  // Minimum enclosing rectangle gives edge direction even for a square, whose
  // covariance has no distinguished axis. Normalise equivalent quarter turns.
  std::vector<cv::Point2f> xy;
  for (const auto& p : inliers) xy.emplace_back(p[0], p[1]);
  cv::Point2f corners[4]; cv::minAreaRect(xy).points(corners);
  const auto edge = corners[1] - corners[0];
  const double yaw = std::remainder(std::atan2(edge.y, edge.x), CV_PI / 2);
  const double c = std::cos(yaw), s = std::sin(yaw);
  xs.clear(); ys.clear();
  for (const auto& p : inliers) {
    xs.push_back(c*p[0]+s*p[1]); ys.push_back(-s*p[0]+c*p[1]);
  }
  const double x0 = quantile(xs, .01), x1 = quantile(xs, .99);
  const double y0 = quantile(ys, .01), y1 = quantile(ys, .99);
  const double width = x1 - x0, depth = y1 - y0;
  if (width < cube_size * .88 || width > cube_size * 1.12 ||
      depth < cube_size * .88 || depth > cube_size * 1.12) return std::nullopt;

  // Require actual planar area, not just a long thin rim whose bounds happen
  // to span a square. Perspective changes pixel density but not XY coverage.
  bool occupied[100] = {};
  for (const auto& p : inliers) {
    const int x = std::clamp(static_cast<int>((c*p[0]+s*p[1] - x0) / width * 10), 0, 9);
    const int y = std::clamp(static_cast<int>((-s*p[0]+c*p[1] - y0) / depth * 10), 0, 9);
    occupied[y * 10 + x] = true;
  }
  const double coverage = std::count(std::begin(occupied), std::end(occupied), true) / 100.0;
  if (coverage < .75) return std::nullopt;
  const double z = quantile(zs, .5);
  return TopFace{cv::Vec3d(c*(x0+x1)/2-s*(y0+y1)/2, s*(x0+x1)/2+c*(y0+y1)/2, z-cube_size/2),
                 z, width, depth, std::sqrt(squared_error / inliers.size()), coverage, yaw};
}
}  // namespace classify_perception
