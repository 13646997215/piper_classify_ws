#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>

namespace classify_task {
struct BinLayout {
  Eigen::Isometry3d floor=Eigen::Isometry3d::Identity();
  double xmin, xmax, ymin, ymax, floor_top, rim;
  std::vector<Eigen::Vector3d> slots(unsigned count, double cube=.05) const {
    if (!count || count>4) throw std::runtime_error("Bin supports 1..4 floor slots");
    const unsigned nx=count>2 ? 2 : 1, ny=count>1 ? 2 : 1;
    const double pitch=cube+.02, margin=.01;
    if (xmax-xmin < nx*cube+(nx-1)*.02+2*margin ||
        ymax-ymin < ny*cube+(ny-1)*.02+2*margin)
      throw std::runtime_error("Bin capacity insufficient for separated cubes and wall margin");
    std::vector<Eigen::Vector3d> out;
    for (unsigned y=0;y<ny;++y) for (unsigned x=0;x<nx && out.size()<count;++x)
      out.push_back(floor*Eigen::Vector3d((xmin+xmax)/2+(x-(nx-1)/2.)*pitch,
        (ymin+ymax)/2+(y-(ny-1)/2.)*pitch,floor_top+cube/2));
    return out;
  }
  bool contains(const Eigen::Isometry3d& object, double cube=.05) const {
    const Eigen::Isometry3d local=floor.inverse()*object;
    for (int x:{-1,1}) for (int y:{-1,1}) for (int z:{-1,1}) {
      const Eigen::Vector3d corner=local*Eigen::Vector3d(x*cube/2,y*cube/2,z*cube/2);
      if (corner.x()<xmin-.001 || corner.x()>xmax+.001 ||
          corner.y()<ymin-.001 || corner.y()>ymax+.001 || corner.z()<floor_top-.002) return false;
    }
    return true;
  }
};
// Separating-axis test for full oriented cubes, with a 2 mm required gap.
inline bool separated(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b,double cube=.05) {
  std::vector<Eigen::Vector3d> axes;
  for (int i=0;i<3;++i) { axes.push_back(a.linear().col(i));axes.push_back(b.linear().col(i)); }
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) axes.push_back(a.linear().col(i).cross(b.linear().col(j)));
  for (auto axis:axes) {
    if (axis.norm()<1e-8) continue;
    axis.normalize();double radius=0;
    for (int i=0;i<3;++i) radius+=cube/2*(std::abs(axis.dot(a.linear().col(i)))+std::abs(axis.dot(b.linear().col(i))));
    if (std::abs(axis.dot(a.translation()-b.translation()))>radius+.002) return true;
  }
  return false;
}
}
