#pragma once

#include <orb-features/feature_point.h>
#include "types.h"

namespace orb_features {
void compute_harris_response(
    const ImageFloat& image,
    std::vector<FeaturePoint>& features,
    int block_size,
    float k
);
}  // namespace orb_features
