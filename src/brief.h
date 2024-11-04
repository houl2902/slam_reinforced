#pragma once
#include <orb-features/feature_point.h>
#include <random>
#include <unordered_set>
#include <vector>
#include "types.h"

namespace orb_features {
class BRIEF {
 public:
  BRIEF(int rows, int cols, int patch_size);
  BRIEF(const BRIEF&) = delete;
  BRIEF& operator=(const BRIEF&) = delete;
  BRIEF(BRIEF&&);
  BRIEF& operator=(BRIEF&&) = delete;

  void
  get_descriptors(const ImageFloat& image, std::vector<FeaturePoint>& features);

 private:
  void fill_random_coords();
  std::pair<int, int> linear_index_to_coords(int index) const;

 private:
  int patch_size_{0};
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_int_distribution<> dist_;
  ImageFloat smoothed_image_;

  std::vector<int> pc0_;
  std::vector<int> pr0_;
  std::vector<int> pc1_;
  std::vector<int> pr1_;
};
}  // namespace orb_features
