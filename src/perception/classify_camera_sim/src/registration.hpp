#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
namespace classify_camera_sim {
using Intrinsics = std::array<double, 4>;  // fx fy cx cy
// Native depth Z is metres. RGB output Z is millimetres, with zero invalid.
// Nominal inter-sensor rotation is identity; translation is RGB origin in depth.
inline cv::Mat register_depth(const cv::Mat& input, const Intrinsics& kd,
                             const Intrinsics& kc, int width, int height,
                             const cv::Vec3d& color_origin, double min_z,
                             double max_z, double stereo_baseline) {
  cv::Mat result(height, width, CV_16UC1, cv::Scalar(0));
  for (int v = 0; v < input.rows; ++v) for (int u = 0; u < input.cols; ++u) {
    const double z = input.at<float>(v, u);
    // A right IR view must also see the point. This models overlap loss near
    // the left edge, not the stereo matching algorithm or its occlusions.
    if (!std::isfinite(z) || z < min_z || z > max_z || u - kd[0]*stereo_baseline/z < 0) continue;
    const cv::Vec3d p((u-kd[2])*z/kd[0]-color_origin[0],
                     (v-kd[3])*z/kd[1]-color_origin[1], z-color_origin[2]);
    if (p[2] <= 0 || p[2] > 65.535) continue;
    const int x = static_cast<int>(std::lround(kc[0]*p[0]/p[2]+kc[2]));
    const int y = static_cast<int>(std::lround(kc[1]*p[1]/p[2]+kc[3]));
    if (x < 0 || x >= width || y < 0 || y >= height) continue;
    const auto mm = static_cast<std::uint16_t>(std::lround(p[2]*1000));
    auto& old = result.at<std::uint16_t>(y,x);
    if (mm && (!old || mm < old)) old = mm;  // foreground z-buffer
  }
  return result;
}
}
