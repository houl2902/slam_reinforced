#include <orb-features/match.h>
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace orb_features {
namespace {
struct DistPair {
  int dist{0};
  int p0{0};
  int p1{0};
};
bool operator>(const DistPair& a, const DistPair& b) {
  return a.dist > b.dist;
}
}  // namespace

std::unordered_map<size_t, size_t> get_matches(
    const DetectionResult& first,
    const DetectionResult& second,
    int desc_dist_threshold,
    int coord_dist_threshold
) {
  if (first.points.empty() || second.points.empty())
    return {};

  auto num_matches = std::min(first.points.size(), second.points.size());

  std::vector<DistPair> distances;
  distances.reserve(num_matches * 2);

  // compare descriptors on the all pyramid layers
  for (size_t i = 0; i < first.points.size(); ++i) {
    for (size_t j = 0; j < second.points.size(); ++j) {
      // hamming distance
      auto dist = static_cast<int>(
          (first.points[i].descriptor ^ second.points[j].descriptor).count()
      );
      if (dist < desc_dist_threshold) {
        distances.emplace_back(
            DistPair{dist, static_cast<int>(i), static_cast<int>(j)}
        );
        std::push_heap(distances.begin(), distances.end(), std::greater<>());
      }
    }
  }

  std::unordered_map<size_t, size_t> result;
  result.reserve(num_matches);

  auto num_elements = distances.size();
  std::unordered_set<int> used_first;
  std::unordered_set<int> used_second;
  for (size_t i = 0, m = 0; i < num_elements; ++i) {
    std::pop_heap(distances.begin(), distances.end(), std::greater<>());
    auto dist_pair = distances.back();
    distances.pop_back();
    if (used_first.count(dist_pair.p0) == 0 &&
        used_second.count(dist_pair.p1) == 0) {
      // check coord distance
      auto x_diff =
          first.points[dist_pair.p0].x - second.points[dist_pair.p1].x;
      auto y_diff =
          first.points[dist_pair.p0].y - second.points[dist_pair.p1].y;
      auto coord_dist =
          static_cast<int>(sqrt(x_diff * x_diff + y_diff * y_diff));
      if (coord_dist < coord_dist_threshold) {
        result[dist_pair.p0] = dist_pair.p1;
        ++m;
        // mark features as matched
        used_first.insert(dist_pair.p0);
        used_second.insert(dist_pair.p1);
      }

      if (m >= num_matches)
        break;
    }
  }
  return result;
}
}  // namespace orb_features
