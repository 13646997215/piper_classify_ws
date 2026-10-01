#include <gtest/gtest.h>
#include "bin_layout.hpp"
TEST(BinLayout, CapacityAndSeparatedSlots)
{
  classify_task::BinLayout bin;bin.xmin=bin.ymin=-.053;bin.xmax=bin.ymax=.053;bin.floor_top=.006;
  EXPECT_EQ(bin.slots(1).size(),1u);EXPECT_THROW(bin.slots(2),std::runtime_error);
  bin.xmin=bin.ymin=-.088;bin.xmax=bin.ymax=.088;
  const auto slots=bin.slots(4);ASSERT_EQ(slots.size(),4u);
  for (size_t i=0;i<slots.size();++i) {
    Eigen::Isometry3d a=Eigen::Isometry3d::Identity();a.translation()=slots[i];EXPECT_TRUE(bin.contains(a));
    for (size_t j=0;j<i;++j) { auto b=a;b.translation()=slots[j];EXPECT_TRUE(classify_task::separated(a,b)); }
  }
}
TEST(BinLayout, ChecksCornersInRotatedBinAndOrientedOverlap)
{
  classify_task::BinLayout bin;bin.xmin=bin.ymin=-.088;bin.xmax=bin.ymax=.088;bin.floor_top=.006;
  bin.floor=Eigen::Translation3d(.52,.27,.012)*Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d a=bin.floor;a.translation()=bin.slots(2)[0];EXPECT_TRUE(bin.contains(a));
  auto b=a;b.translation()=bin.floor*Eigen::Vector3d(.08,0,.031);EXPECT_FALSE(bin.contains(b));
  b=a;b.translate(Eigen::Vector3d(.035,0,0));EXPECT_FALSE(classify_task::separated(a,b));
  b=a;b.translate(Eigen::Vector3d(.07,0,0));EXPECT_TRUE(classify_task::separated(a,b));
}
