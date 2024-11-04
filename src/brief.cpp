#include "brief.h"
#include "blur.h"

#include <set>
#include <unordered_set>
namespace orb_features {
BRIEF::BRIEF(int rows, int cols, int patch_size)
    : patch_size_(patch_size),
      gen_(rd_()),
      dist_(0, patch_size * patch_size - 1),
      smoothed_image_(rows, cols),
      pc0_(FeaturePoint::descriptor_size),
      pr0_(FeaturePoint::descriptor_size),
      pc1_(FeaturePoint::descriptor_size),
      pr1_(FeaturePoint::descriptor_size) {
  if (static_cast<size_t>(patch_size_) * patch_size_ <
      FeaturePoint::descriptor_size) {
    throw std::logic_error(
        "BRIEF patch size is too small for 256 descriptor size"
    );
  }
}

BRIEF::BRIEF(BRIEF&& other)
    : patch_size_(other.patch_size_),
      gen_(rd_()),
      dist_(std::move(other.dist_)),
      smoothed_image_(std::move(other.smoothed_image_)),
      pc0_(FeaturePoint::descriptor_size),
      pr0_(FeaturePoint::descriptor_size),
      pc1_(FeaturePoint::descriptor_size),
      pr1_(FeaturePoint::descriptor_size) {
  if (static_cast<size_t>(patch_size_) * patch_size_ <
      FeaturePoint::descriptor_size) {
    throw std::logic_error(
        "BRIEF patch size is too small for 256 descriptor size"
    );
  }
}

std::pair<int, int> BRIEF::linear_index_to_coords(int index) const {
  auto row = index / patch_size_;
  auto col = index - row * patch_size_;
  return std::make_pair(row - patch_size_ / 2, col - patch_size_ / 2);
}

void BRIEF::fill_random_coords() {
  std::set<std::pair<int, int>> coord_pairs;
  constexpr size_t len = FeaturePoint::descriptor_size;
  while (coord_pairs.size() != len) {
    coord_pairs.emplace(std::make_pair(dist_(gen_), dist_(gen_)));
  }

  auto pairs_it = coord_pairs.begin();
  for (size_t i = 0; i < len; ++i) {
    std::tie(pr0_[i], pc0_[i]) = linear_index_to_coords(pairs_it->first);
    std::tie(pr1_[i], pc1_[i]) = linear_index_to_coords(pairs_it->second);
    ++pairs_it;
  }
}

void BRIEF::get_descriptors(
    const ImageFloat& image,
    std::vector<FeaturePoint>& features
) {
  constexpr size_t len = FeaturePoint::descriptor_size;
  gaussian_blur(image, smoothed_image_, Gaussian3);

  // generate random coordinates in a patch
  fill_random_coords();

  // calculate descriptors
  for (size_t i = 0; i < features.size(); ++i) {
    auto& descriptor = features[i].descriptor;
    descriptor.reset();
    auto angle = features[i].orientation;
    auto sin_theta = sin(angle);
    auto cos_theta = cos(angle);

    auto fr = features[i].y;
    auto fc = features[i].x;
    for (size_t j = 0; j < len; ++j) {
      // Rotation is based on the idea that:
      // c` = c*cos(th) - r*sin(th)
      // r` = c*sin(th) + r*cos(th)
      auto spr0 =
          static_cast<int>(round(sin_theta * pr0_[j] + cos_theta * pc0_[j]));
      auto spc0 =
          static_cast<int>(round(cos_theta * pr0_[j] - sin_theta * pc0_[j]));
      auto spr1 =
          static_cast<int>(round(sin_theta * pr1_[j] + cos_theta * pc1_[j]));
      auto spc1 =
          static_cast<int>(round(cos_theta * pr1_[j] - sin_theta * pc1_[j]));

      float i0 = 0;
      if ((fr + spr0) >= 0 && (fr + spr0) < smoothed_image_.rows() &&
          (fc + spc0) >= 0 && (fc + spc0) < smoothed_image_.cols()) {
        i0 = smoothed_image_(fr + spr0, fc + spc0);
      }
      float i1 = 0;
      if ((fr + spr1) >= 0 && (fr + spr1) < smoothed_image_.rows() &&
          (fc + spc1) >= 0 && (fc + spc1) < smoothed_image_.cols()) {
        i1 = smoothed_image_(fr + spr1, fc + spc1);
      }
      if (i0 < i1)
        descriptor[i] = true;
    }
  }
}
}  // namespace orb_features
