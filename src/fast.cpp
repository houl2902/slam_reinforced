#include "fast.h"
#include "types.h"

#include <limits>
#include <set>

namespace orb_features {
namespace {
const Eigen::Array4i cross_indices = {3, 21, 27, 45};

// sequentially ordered
const Eigen::Array<int, 16, 1, 0> circle_indices =
    {2, 3, 4, 12, 20, 27, 34, 40, 46, 45, 44, 36, 28, 21, 14, 8};

}  // namespace
std::vector<FeaturePoint> fast(
    const ImageFloat& image,
    int contiguous_num,
    float threshold,
    int nms_window
) {
  std::vector<FeaturePoint> features;

  ImageFloat corners = ImageFloat::Zero(image.rows(), image.cols());

  FeaturePoint feature;
  for (int y = 3; y < image.rows() - 3; ++y) {
    for (int x = 3; x < image.cols() - 3; ++x) {
      float center_value = image(y, x);
      auto t = threshold < 1.f ? threshold * center_value : threshold;
      auto pixel_patch =
          image(Eigen::seq(y - 3, y + 3), Eigen::seq(x - 3, x + 3))
              .reshaped()
              .array();

      int higher_intensity_num = 0;
      int max_higher_intensity_num = 0;
      Eigen::Index last_high_index = circle_indices[0];

      int lower_intensity_num = 0;
      int max_lower_intensity_num = 0;
      Eigen::Index last_low_index = circle_indices[0];

      for (Eigen::Index i = 0; i < circle_indices.size(); ++i) {
        if (pixel_patch[circle_indices[i]] > (center_value + t)) {
          if (i == 0 or last_high_index == circle_indices[i - 1]) {
            ++higher_intensity_num;
          } else {
            max_higher_intensity_num =
                std::max(max_higher_intensity_num, higher_intensity_num);
            higher_intensity_num = 1;
          }
          last_high_index = circle_indices[i];
        }

        else if (pixel_patch[circle_indices[i]] < (center_value - t)) {
          if (i == 0 or last_low_index == circle_indices[i - 1]) {
            ++lower_intensity_num;
          } else {
            max_lower_intensity_num =
                std::max(max_lower_intensity_num, lower_intensity_num);
            lower_intensity_num = 1;
          }
          last_low_index = circle_indices[i];
        }
      }

      if (max_higher_intensity_num >= contiguous_num ||
          max_lower_intensity_num >= contiguous_num) {
        feature.x = x;
        feature.y = y;
        features.push_back(feature);
        corners(y, x) =
            (center_value - pixel_patch.array()(circle_indices)).abs().sum();
      }
    }
  }

  // Non Maximal Suppression
  if (nms_window != 0) {
    auto nms_window_half = nms_window / 2;
    std::vector<FeaturePoint> nms_features;
    std::set<std::pair<int, int>> existing_features;
    std::pair<int, int> feature_key;
    for (auto feature : features) {
      auto wy_start = std::max(feature.y - nms_window_half, 0);
      auto wy_end = std::min(
          feature.y + nms_window_half,
          static_cast<int>(corners.rows() - 1)
      );
      auto wx_start = std::max(feature.x - nms_window_half, 0);
      auto wx_end = std::min(
          feature.x + nms_window_half,
          static_cast<int>(corners.cols() - 1)
      );
      auto window =
          corners(Eigen::seq(wy_start, wy_end), Eigen::seq(wx_start, wx_end));

      Eigen::Index y_new, x_new;
      window.maxCoeff(&y_new, &x_new);
      feature.y = feature.y + y_new - nms_window_half;
      feature.x = feature.x + x_new - nms_window_half;

      feature_key.first = feature.y;
      feature_key.second = feature.x;
      if (existing_features.count(feature_key) == 0) {
        existing_features.insert(feature_key);
        nms_features.push_back(std::move(feature));
      }
    }
    return nms_features;
  }
  return features;
}
}  // namespace orb_features
