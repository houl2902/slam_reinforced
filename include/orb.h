#pragma once
#include <orb-features/feature_point.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace orb_features {
struct DetectionResult {
  std::vector<FeaturePoint> points;
};

struct OrbSettings {
  int n_features = 500;

  int scale_pyramid_levels = 8;
  float pyramid_scale_factor = 1.2f;

  int fast_contiguous_num = 12; // corner angle size, circle sector
  float fast_threshold = 0.15f; // difference in pixel intensities
  int fast_nms_window = 9;

  int harris_block_size = 9;
  float harris_k = 0.04f; // 0.06

  int orientation_block_size = 33;
  int brief_block_size = 33;
};

class ORB {
 public:
  ORB(int rows, int cols, OrbSettings settings = {});

  ORB(const ORB&) = delete;
  ORB& operator=(const ORB&) = delete;
  ORB(ORB&&);
  ORB& operator=(ORB&&);
  ~ORB();

  DetectionResult get_orb_features(const uint8_t* image_data) const;

  std::pair<int, int> coords_to_zero_level(int level, int x, int y) const;

  std::tuple<int, int, const float*> image(int level) const;

  const OrbSettings& settings() const;

 private:
  void init_features_per_level();

 private:
  int rows_{0};
  int cols_{0};
  OrbSettings settings_;
  struct ORBDetails;
  std::unique_ptr<ORBDetails> details_;
  std::vector<size_t> features_per_level_;
};

}  // namespace orb_features
