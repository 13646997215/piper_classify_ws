#include <gtest/gtest.h>
#include "scene_tracker.hpp"

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = .028; return p;
}

TEST(SceneTracker, ConfirmsThreeFramesAndRejectsOneFrameJump)
{
  classify_scene_bridge::ObservationTrack track;
  const auto home = point(.3, 0);
  EXPECT_FALSE(track.observe(home, 1));
  EXPECT_FALSE(track.observe(home, 1.2));
  EXPECT_TRUE(track.observe(home, 1.4));
  EXPECT_TRUE(track.fresh(1.5));
  EXPECT_FALSE(track.observe(point(.28, .025), 1.6));
  EXPECT_FALSE(track.fresh(1.7));
  EXPECT_DOUBLE_EQ(track.accepted.x, .3);
  track.missing();
  EXPECT_DOUBLE_EQ(track.accepted.x, .3);
  EXPECT_FALSE(track.fresh(1.8));
  EXPECT_FALSE(track.observe(home, 1.8));
  EXPECT_TRUE(track.fresh(1.9));
}

TEST(SceneTracker, AcceptsRealDisplacementAndExpiresMissingStream)
{
  classify_scene_bridge::ObservationTrack track;
  const auto p = point(.28, .035);
  track.observe(point(.3, 0), 1); track.observe(point(.3, 0), 1.2); track.observe(point(.3, 0), 1.4);
  EXPECT_FALSE(track.observe(p, 2));
  EXPECT_FALSE(track.observe(p, 2.2));
  EXPECT_TRUE(track.observe(p, 2.4));
  EXPECT_NEAR(track.accepted.x, .28, 1e-9);
  EXPECT_NEAR(track.accepted.y, .035, 1e-9);
  EXPECT_TRUE(track.fresh(2.6));
  EXPECT_FALSE(track.fresh(3.3));
}

TEST(SceneTracker, QuarterTurnEquivalentAndRotationNeedsConfirmation)
{
  classify_scene_bridge::ObservationTrack t;const auto p=point(.4,0);
  t.observe(p,1,.76);t.observe(p,1.2,.76);ASSERT_TRUE(t.observe(p,1.4,.76));
  EXPECT_FALSE(t.observe(p,1.6,.76-M_PI/2));EXPECT_TRUE(t.fresh(1.7));
  EXPECT_FALSE(t.observe(p,1.8,.20));EXPECT_FALSE(t.fresh(1.9));
  t.observe(p,2.,.20);EXPECT_TRUE(t.observe(p,2.2,.20));EXPECT_NEAR(t.accepted_yaw,.2,1e-8);
}
TEST(SceneAssociation, ReorderedSameColourAndAmbiguity)
{
  using C=classify_scene_bridge::TrackCandidate;
  const std::vector<C> tracks={{"red",point(.32,0)},{"red",point(.4,0)}};
  EXPECT_EQ(classify_scene_bridge::associate({{"red",point(.401,0)},{"red",point(.321,0)}},tracks),(std::vector<int>{1,0}));
  EXPECT_EQ(classify_scene_bridge::associate({{"red",point(.36,0)}},tracks),(std::vector<int>{-2}));
  EXPECT_EQ(classify_scene_bridge::associate({{"blue",point(.32,0)}},tracks),(std::vector<int>{-1}));
  const auto duplicate=classify_scene_bridge::associate({{"red",point(.32,0)},{"red",point(.321,0)}},tracks);
  EXPECT_EQ(duplicate,(std::vector<int>{0,-2}));
}

TEST(SceneTracker, AllowsOnlyAlreadyAcceptedSmallClockLead)
{
  classify_scene_bridge::ObservationTrack t;const auto p=point(.4,0);
  t.observe(p,1);t.observe(p,1.2);t.observe(p,1.4);
  EXPECT_TRUE(t.fresh(1.38));EXPECT_FALSE(t.fresh(1.30));
}
