#pragma once
#include <cmath>
#include <random>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace classify_camera_sim {
// Controlled stress input, not a calibrated Gemini noise model.
class DepthStress {
public:
  DepthStress(double stddev, double dropout, unsigned seed)
    : stddev_(stddev), dropout_(dropout), random_(seed) {
    if (!std::isfinite(stddev) || stddev<0 || stddev>.01 ||
        !std::isfinite(dropout) || dropout<0 || dropout>.5)
      throw std::invalid_argument("Depth stress expects 0..10 mm stddev and 0..50% dropout");
  }
  bool enabled() const { return stddev_>0 || dropout_>0; }
  cv::Mat apply(const cv::Mat& source) {
    if (!enabled()) return source;
    if (source.type()!=CV_32FC1) throw std::invalid_argument("Native depth stress expects 32FC1 metres");
    cv::Mat result=source.clone();
    std::normal_distribution<double> noise(0.,stddev_>0 ? stddev_ : 1.);
    std::uniform_real_distribution<double> probability(0.,1.);
    for (int row=0;row<result.rows;++row) for (int column=0;column<result.cols;++column) {
      auto& z=result.at<float>(row,column);
      if (!std::isfinite(z) || z<=0) continue;
      if (probability(random_)<dropout_) z=0;
      else if (stddev_>0) z=std::max(0.,static_cast<double>(z)+noise(random_));
    }
    return result;
  }
private:
  double stddev_,dropout_;std::mt19937 random_;
};
}
