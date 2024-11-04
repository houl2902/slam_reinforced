#pragma once

#include <orb-features/feature_point.h>
#include <stddef.h>
#include <vector>

namespace orb_features {
void retain_with_best_response(
    std::vector<FeaturePoint>& features,
    size_t n_points
);
}  // namespace orb_features
