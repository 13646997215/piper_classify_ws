#pragma once
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace classify_scene_bridge
{
inline double distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

inline double yaw_delta(double a, double b) { return std::remainder(a-b, M_PI/2); }

struct ObservationTrack
{
  geometry_msgs::msg::Point accepted, pending;
  bool known{false}, visible{false};
  unsigned count{0};
  double first_pending{0}, last_seen{0}, accepted_yaw{0}, pending_yaw{0};

  void missing() { visible = false; count = 0; }

  bool observe(const geometry_msgs::msg::Point& p, double stamp, double yaw=0)
  {
    if (known && distance(p, accepted) < .003 && std::abs(yaw_delta(yaw, accepted_yaw)) < .025) {
      visible = true; last_seen = stamp; count = 0;
      return false;
    }
    visible = false;
    if (count == 0 || distance(p, pending) > .003 || std::abs(yaw_delta(yaw, pending_yaw)) > .025 || stamp - last_seen > .6) {
      pending = p; pending_yaw=yaw; count = 1; first_pending = stamp;
    } else {
      pending.x = (pending.x * count + p.x) / (count + 1);
      pending.y = (pending.y * count + p.y) / (count + 1);
      pending.z = (pending.z * count + p.z) / (count + 1);
      pending_yaw=std::remainder(pending_yaw+yaw_delta(yaw,pending_yaw)/(count+1),M_PI/2);
      ++count;
    }
    last_seen = stamp;
    if (count < 3 || stamp - first_pending < .35) return false;
    accepted = pending; accepted_yaw=pending_yaw; known = visible = true; count = 0;
    return true;
  }

  bool fresh(double stamp) const
  {
    return known && visible && stamp + .05 >= last_seen && stamp - last_seen < .8;
  }
};
// Gated nearest-neighbour association; each observation/track is used once.
// -1 is a genuinely new object; -2 is an ambiguous/conflicting observation
// that must never create a second identity or be used for a pickup.
struct TrackCandidate { std::string color; geometry_msgs::msg::Point position; };
inline std::vector<int> associate(const std::vector<TrackCandidate>& observations,
                                  const std::vector<TrackCandidate>& tracks) {
  std::vector<int> result(observations.size(), -1);
  std::vector<std::pair<double,size_t>> order;
  for (size_t i=0;i<observations.size();++i) {
    double best=.045, second=.045; int index=-1;
    for (size_t j=0;j<tracks.size();++j) if (observations[i].color==tracks[j].color) {
      const double d=distance(observations[i].position,tracks[j].position);
      if (d<best) { second=best;best=d;index=j; } else second=std::min(second,d);
    }
    if (index>=0) { result[i]=(second<.045 && second-best<.010) ? -2 : index; order.emplace_back(best,i); }
  }
  std::sort(order.begin(),order.end()); std::vector<bool> used(tracks.size(),false);
  for (const auto& item:order) {
    int& j=result[item.second];if (j<0) continue;
    if (used[j]) j=-2;else used[j]=true;
  }
  return result;
}
}  // namespace classify_scene_bridge
