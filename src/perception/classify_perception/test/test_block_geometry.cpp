#include <gtest/gtest.h>
#include "block_geometry.hpp"

namespace
{
std::vector<cv::Vec3d> top(double cx, double cy, double width = .05)
{
  std::vector<cv::Vec3d> points;
  for (int i = 0; i <= 50; ++i) {
    for (int j = 0; j <= 50; ++j) {
      points.emplace_back(cx - .025 + width * i / 50, cy - .025 + .05 * j / 50, .053);
    }
  }
  return points;
}
}

TEST(BlockGeometry, FullTopAndNonuniformPerspectiveGiveSameCentre)
{
  auto points = top(.28, .035);
  // Mimic a nearer edge producing many more pixels, plus a visible side face.
  for (int repeat = 0; repeat < 6; ++repeat) {
    for (int i = 0; i <= 50; ++i) {
      for (int j = 0; j < 15; ++j) points.emplace_back(.255 + .001*i, .010 + .001*j, .053);
      for (int j = 0; j < 45; ++j) points.emplace_back(.255 + .001*i, .010, .003 + .001*j);
    }
  }
  const auto fit = classify_perception::fit_top_face(points);
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->center[0], .28, .002);
  EXPECT_NEAR(fit->center[1], .035, .002);
  EXPECT_NEAR(fit->center[2], .028, .001);
}

TEST(BlockGeometry, RejectsSideOnlyAndCroppedTop)
{
  std::vector<cv::Vec3d> side;
  for (int i = 0; i < 50; ++i) for (int j = 0; j < 50; ++j)
    side.emplace_back(.275 + i * .001, .115, .003 + j * .001);
  EXPECT_FALSE(classify_perception::fit_top_face(side));
  EXPECT_FALSE(classify_perception::fit_top_face(top(.3, .14, .028)));
}

TEST(BlockGeometry, RejectsWrongHeightAndRim)
{
  auto points = top(.3, .14);
  for (auto& p : points) p[2] -= .02;
  EXPECT_FALSE(classify_perception::fit_top_face(points));
  auto full = top(.3, .14);
  std::vector<cv::Vec3d> rim;
  for (const auto& p : full) if (p[0] < .277 || p[0] > .323 || p[1] < .117 || p[1] > .163) rim.push_back(p);
  EXPECT_FALSE(classify_perception::fit_top_face(rim));
}

TEST(BlockGeometry, RotatedSquareAndCroppedSquare)
{
  for (const double angle : {-.70,-.30,.25,.72,1.40}) {
    auto points=top(0,0);
    for (auto& p:points) {
      const double x=p[0],y=p[1];p[0]=.4+std::cos(angle)*x-std::sin(angle)*y;
      p[1]=.06+std::sin(angle)*x+std::cos(angle)*y;
    }
    const auto fit=classify_perception::fit_top_face(points);ASSERT_TRUE(fit);
    EXPECT_NEAR(fit->center[0],.4,.001);EXPECT_NEAR(fit->center[1],.06,.001);
    EXPECT_NEAR(std::remainder(fit->yaw-angle,CV_PI/2),0,.005);
    points.erase(std::remove_if(points.begin(),points.end(),[&](const auto& p) {
      return std::cos(angle)*(p[0]-.4)+std::sin(angle)*(p[1]-.06)>.005;
    }),points.end());
    EXPECT_FALSE(classify_perception::fit_top_face(points));
  }
}
