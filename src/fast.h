#pragma once
#include <orb-features/feature_point.h>
#include <vector>
#include "types.h"

namespace orb_features {
std::vector<FeaturePoint> fast(
    const ImageFloat& image,
    int contiguous_num,
    float threshold,
    int nms_window
);
}  // namespace orb_features
