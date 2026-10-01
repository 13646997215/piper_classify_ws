#include <gtest/gtest.h>
#include "registration.hpp"
#include "depth_stress.hpp"
TEST(Registration, AppliesOpticalOffsetAndMillimetreUnits) {
  cv::Mat image(5,7,CV_32FC1,cv::Scalar(0)); image.at<float>(2,3)=1;
  auto out=classify_camera_sim::register_depth(image,{10,10,3,2},{10,10,3,2},7,5,{.1,0,.002},.17,20,0);
  EXPECT_EQ(out.at<std::uint16_t>(2,2),998);
  EXPECT_EQ(out.at<std::uint16_t>(2,3),0);
}
TEST(Registration, RejectsTooCloseDepthAndStereoNonOverlap) {
  cv::Mat image(5,7,CV_32FC1,cv::Scalar(0));
  image.at<float>(2,3)=.1;image.at<float>(2,0)=.2;
  auto out=classify_camera_sim::register_depth(image,{10,10,3,2},{10,10,3,2},7,5,{0,0,0},.17,20,.05);
  EXPECT_EQ(cv::countNonZero(out),0);
}
TEST(Registration, KeepsForegroundWhenMultipleDepthPixelsMapToOneColorPixel) {
  cv::Mat image(1,3,CV_32FC1,cv::Scalar(0));
  image.at<float>(0,0)=2; image.at<float>(0,1)=1;
  auto out=classify_camera_sim::register_depth(image,{10,10,0,0},{1,1,0,0},3,1,{0,0,0},.17,20,0);
  EXPECT_EQ(out.at<std::uint16_t>(0,0),1000);
  EXPECT_EQ(cv::countNonZero(out),1);
}

TEST(DepthStress, UsesMetresAndPreservesInvalidDepth) {
  cv::Mat source(128,128,CV_32FC1,cv::Scalar(1.));source.at<float>(0,0)=0;
  classify_camera_sim::DepthStress stress(.001,0.,17);
  auto out=stress.apply(source);cv::Scalar mean,stddev;cv::meanStdDev(out(cv::Rect(1,1,127,127)),mean,stddev);
  EXPECT_NEAR(mean[0],1.,.00005);EXPECT_NEAR(stddev[0],.001,.00005);
  EXPECT_EQ(out.at<float>(0,0),0.);EXPECT_EQ(source.at<float>(1,1),1.);
}

TEST(DepthStress, DropoutCanBeReproducedWithoutChangingBaseline) {
  cv::Mat source(100,100,CV_32FC1,cv::Scalar(.4));
  classify_camera_sim::DepthStress first(0.,.1,5),second(0.,.1,5),baseline(0.,0.,5);
  auto a=first.apply(source),b=second.apply(source);
  EXPECT_EQ(cv::countNonZero(a!=b),0);
  EXPECT_NEAR(1.-cv::countNonZero(a)/10000.,.1,.015);
  EXPECT_EQ(cv::countNonZero(baseline.apply(source)!=source),0);
}
