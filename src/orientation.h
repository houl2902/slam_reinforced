#pragma once
#include <orb-features/feature_point.h>
#include <vector>
#include "types.h"

namespace orb_features {
class FeatureOrientation {
 public:
  FeatureOrientation(int patch_size);

  void calculate_orientations(
      const ImageFloat& image,
      std::vector<FeaturePoint>& features
  ) const;

 private:
  int patch_size_{0};
  int mask_rows2_{0};
  int mask_cols2_{0};
  ImageFloat mask_;
};
}  // namespace orb_features
