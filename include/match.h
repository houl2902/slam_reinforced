#pragma once
#include <orb-features/orb.h>
#include <limits>
#include <unordered_map>

namespace orb_features {
std::unordered_map<size_t, size_t> get_matches(
    const DetectionResult& first,
    const DetectionResult& second,
    int desc_dist_threshold = std::numeric_limits<int>::max(),
    int coord_dist_threshold = std::numeric_limits<int>::max()
);
}  // namespace orb_features
