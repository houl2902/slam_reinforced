#include "orientation.h"

namespace orb_features {
namespace {

void draw_circle_part(ImageFloat& img, int xc, int yc, int x, int y) {
  img(yc + y, xc + x) = 1;
  img(yc + y, xc - x) = 1;
  img(yc - y, xc + x) = 1;
  img(yc - y, xc - x) = 1;
  img(yc + x, xc + y) = 1;
  img(yc + x, xc - y) = 1;
  img(yc - x, xc + y) = 1;
  img(yc - x, xc - y) = 1;
}

void make_circle(ImageFloat& img, int xc, int yc, int r) {
  int x = 0;
  auto y = r;
  auto d = 3 - 2 * r;
  draw_circle_part(img, xc, yc, x, y);
  while (y >= x) {
    ++x;
    if (d > 0) {
      y -= 1;
      d = d + 4 * (x - y) + 10;
    } else {
      d = d + 4 * x + 6;
    }
    draw_circle_part(img, xc, yc, x, y);
  }
}

void fill_circle(ImageFloat& img) {
  for (int y = 1; y < img.rows() - 1; ++y) {
    bool do_fill = false;
    for (int x = 0; x < img.cols(); ++x) {
      if (img(y, x) == 1)
        do_fill = !do_fill;
      else if (do_fill)
        img(y, x) = 2;
    }
  }
}
}  // namespace

FeatureOrientation::FeatureOrientation(int patch_size)
    : patch_size_(patch_size) {
  if (patch_size_ % 2 == 0)
    patch_size_ -= 1;  // make odd

  mask_ = ImageFloat(patch_size_, patch_size_);
  mask_rows2_ = (mask_.rows() - 1) / 2;
  mask_cols2_ = (mask_.cols() - 1) / 2;

  int r = patch_size_ / 2;
  make_circle(mask_, r, r, r);
  fill_circle(mask_);
}

void FeatureOrientation::calculate_orientations(
    const ImageFloat& image,
    std::vector<FeaturePoint>& features
) const {
  std::vector<float> orientations;
  orientations.reserve(features.size());

  for (size_t i = 0; i < features.size(); ++i) {
    auto c0 = features[i].x;
    auto r0 = features[i].y;
    float m01 = 0;
    float m10 = 0;
    for (int r = 0; r < mask_.rows(); ++r) {
      auto m01_temp = 0;
      for (int c = 0; c < mask_.cols(); ++c) {
        if (mask_(r, c) > 0) {
          float intensity = 0;
          auto im_r = r0 + r - mask_rows2_;
          auto im_c = c0 + c - mask_cols2_;
          if (im_r >= 0 && im_r < image.rows() && im_c >= 0 &&
              im_c < image.cols()) {
            intensity = image(im_r, im_c);
          }
          m10 = m10 + intensity * (c - mask_cols2_);
          m01_temp = m01_temp + intensity;
        }
      }
      m01 = m01 + m01_temp * (r - mask_rows2_);
    }
    features[i].orientation = static_cast<float>(atan2(m01, m10));
  }
}

}  // namespace orb_features
