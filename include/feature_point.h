#pragma once
#include <bitset>
#include <vector>

namespace orb_features {
struct FeaturePoint {
  static constexpr size_t descriptor_size = 256;
  int x{0};
  int y{0};
  int level{0};       // scale pyramid level
  float response{1};  // value to compare features quality
  float orientation{0};
  std::bitset<descriptor_size> descriptor;
};
}  // namespace orb_features