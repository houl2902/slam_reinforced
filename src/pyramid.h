#pragma once
#include <array>
#include <vector>
#include "types.h"

namespace orb_features {

class ScalePyramid {
 public:
  ScalePyramid(int rows, int cols, int levels, float scale_factor);

  void fill(const ImageFloat& image);

  int levels() const;

  const ImageFloat& image(int level) const;
  std::pair<int, int> image_size(int level) const;

  float scale_factor() const;

  std::pair<int, int> coords_to_zero_level(int level, int x, int y) const;

 private:
  void fill_bilinear_scale(const ImageFloat& image);
  void fill_average_scale(const ImageFloat& image);

 private:
  int levels_{1};
  float scale_factor_{1.2f};
  std::vector<ImageFloat> images_;
};

}  // namespace orb_features
