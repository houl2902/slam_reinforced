#include "point_filter.h"
#include <algorithm>

namespace orb_features {
void retain_with_best_response(
    std::vector<FeaturePoint>& features,
    size_t n_points
) {
  if (features.size() > n_points) {
    if (n_points == 0) {
      features.clear();
      return;
    }
    // first use nth element to partition the features into the best and worst.
    std::nth_element(
        features.begin(),
        features.begin() + n_points - 1,
        features.end(),
        [](auto& a, auto& b) { return a.response > b.response; }
    );
    // this is the boundary response, and in the case of FAST may be ambiguous
    float ambiguous_response = features[n_points - 1].response;
    // use std::partition to grab all of the features with the boundary
    // response.
    auto new_end = std::partition(
        features.begin() + n_points,
        features.end(),
        [ambiguous_response](auto& fp) {
          return fp.response >= ambiguous_response;
        }
    );
    // resize the features, given this new end point. nth_element and partition
    // reordered the points inplace
    features.resize(new_end - features.begin());
  }
}
}  // namespace orb_features
