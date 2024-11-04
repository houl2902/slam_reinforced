#include <orb-features/orb.h>
#include "brief.h"
#include "fast.h"
#include "harris.h"
#include "orientation.h"
#include "point_filter.h"
#include "pyramid.h"

#include <future>

namespace orb_features {
struct ORB::ORBDetails {
  ORBDetails(int rows, int cols, OrbSettings settings)
      : pyramid_(
            rows,
            cols,
            settings.scale_pyramid_levels,
            settings.pyramid_scale_factor
        ),
        orientation_(settings.orientation_block_size) {
    for (int l = 0; l < pyramid_.levels(); ++l) {
      auto img_size = pyramid_.image_size(l);
      brief_levels_.emplace_back(
          BRIEF(img_size.first, img_size.second, settings.brief_block_size)
      );
    }
  }
  ScalePyramid pyramid_;
  FeatureOrientation orientation_;
  std::vector<BRIEF> brief_levels_;
};

ORB::~ORB() {}

ORB::ORB(int rows, int cols, OrbSettings settings)
    : rows_(rows),
      cols_(cols),
      settings_(settings),
      details_(std::make_unique<ORBDetails>(rows, cols, settings)) {
  settings.scale_pyramid_levels = details_->pyramid_.levels();
  init_features_per_level();
}

ORB::ORB(ORB&& other)
    : rows_(other.rows_),
      cols_(other.cols_),
      settings_(std::move(other.settings_)),
      details_(std::move(other.details_)),
      features_per_level_(std::move(other.features_per_level_)) {}

ORB& ORB::operator=(ORB&& other) {
  if (this == &other)
    return *this;
  rows_ = other.rows_;
  cols_ = other.cols_;
  settings_ = std::move(other.settings_);
  details_ = std::move(other.details_);
  features_per_level_ = std::move(other.features_per_level_);
  return *this;
}

std::tuple<int, int, const float*> ORB::image(int level) const {
  const auto& image = details_->pyramid_.image(level);
  return {image.rows(), image.cols(), image.data()};
}

const OrbSettings& ORB::settings() const {
  return settings_;
}

std::pair<int, int> ORB::coords_to_zero_level(int level, int x, int y) const {
  return details_->pyramid_.coords_to_zero_level(level, x, y);
}

void ORB::init_features_per_level() {
  float factor = 1.f / details_->pyramid_.scale_factor();
  auto levels = details_->pyramid_.levels();
  float features_per_scale = static_cast<float>(
      settings_.n_features * (1. - factor) /
      (1. - std::pow(static_cast<double>(factor), static_cast<double>(levels)))
  );

  features_per_level_.resize(levels);
  int sum_features = 0;
  for (int l = 0; l < levels - 1; ++l) {
    features_per_level_[l] = std::round(features_per_scale);
    sum_features += features_per_level_[l];
    features_per_scale *= factor;
  }
  features_per_level_[levels - 1] =
      std::max(settings_.n_features - sum_features, 0);
}

DetectionResult ORB::get_orb_features(const uint8_t* image_data) const {
  DetectionResult result;

  Eigen::Map<const Image> image(image_data, rows_, cols_);
  ImageFloat image_f = image.cast<float>();
  details_->pyramid_.fill(image_f);

  std::vector<std::future<std::vector<FeaturePoint>>> futures;
  for (int l = 0; l < details_->pyramid_.levels(); ++l) {
    auto future = std::async(std::launch::async, [l, this]() {
      auto& level_image = details_->pyramid_.image(l);
      // 1. detect features
      auto features = fast(
          level_image,
          settings_.fast_contiguous_num,
          settings_.fast_threshold,
          settings_.fast_nms_window
      );
      for (auto& feature : features) {
        feature.level = l;
      }

      // 2. compute harris response
      compute_harris_response(
          level_image,
          features,
          settings_.harris_block_size,
          settings_.harris_k
      );

      // 3. filter the most interesting features
      retain_with_best_response(features, features_per_level_[l]);

      // 4. compute orientation
      details_->orientation_.calculate_orientations(level_image, features);

      // 5. compute rBRIEF descriptors
      details_->brief_levels_[l].get_descriptors(level_image, features);

      return features;
    });
    futures.emplace_back(std::move(future));
  }

  // 6. combine results
  for (auto& future : futures) {
    auto features = future.get();
    using IterType = std::vector<FeaturePoint>::iterator;
    result.points.insert(
        result.points.end(),
        std::move_iterator<IterType>(features.begin()),
        std::move_iterator<IterType>(features.end())
    );
  }

  return result;
}

}  // namespace orb_features
