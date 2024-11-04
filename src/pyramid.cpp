#include <cmath>
#include "blur.h"
#include "pyramid.h"

namespace orb_features {

static constexpr int MIN_SIZE = 10;

ScalePyramid::ScalePyramid(int rows, int cols, int levels, float scale_factor)
    : levels_(levels), scale_factor_(scale_factor) {
  float inv_scale_factor = 1.f / scale_factor_;
  images_.push_back(ImageFloat(rows, cols));
  for (int l = 1; l < levels; ++l) {
    auto new_rows = static_cast<int>(rows * std::pow(inv_scale_factor, l));
    auto new_cols = static_cast<int>(cols * std::pow(inv_scale_factor, l));
    if (new_rows >= MIN_SIZE && new_cols >= MIN_SIZE) {
      images_.push_back(ImageFloat(new_rows, new_cols));
    } else {
      break;
    }
  }
  levels_ = static_cast<int>(images_.size());
}

int ScalePyramid::levels() const {
  return levels_;
}

const ImageFloat& ScalePyramid::image(int level) const {
  return images_.at(level);
}

std::pair<int, int> ScalePyramid::image_size(int level) const {
  return {images_.at(level).rows(), images_.at(level).cols()};
}

void ScalePyramid::fill(const ImageFloat& image) {
  // fill_bilinear_scale(image);
  fill_average_scale(image);
}

void ScalePyramid::fill_average_scale(const ImageFloat& image) {
  images_[0] = image;
  Eigen::ArrayXf x;
  Eigen::ArrayXf y;

  for (int l = 1; l < levels_; ++l) {
    // calculate real scale factor after dims ceiling
    auto scale_factor_x =
        1.f / (static_cast<float>(images_[l].cols()) / images_[0].cols());
    auto scale_factor_y =
        1.f / (static_cast<float>(images_[l].rows()) / images_[0].rows());

    // X interpolation
    // calculate corresponding original image coords
    auto scaled_cols = images_[l].cols();
    x.resize(scaled_cols);
    for (Eigen::Index i = 0; i < scaled_cols; ++i) {
      auto x_scaled = i * scale_factor_x;
      x[i] = x_scaled;
    }
    // interpolation
    ImageFloat x_inter(image.rows(), scaled_cols);
    for (Eigen::Index row = 0; row < x_inter.rows(); ++row) {
      for (Eigen::Index col = 0; col < scaled_cols; ++col) {
        // average range for single pixel
        float start = x[col];
        float end = (col + 1) < scaled_cols ? x[col + 1] : image.cols() - 1;

        int i_start = static_cast<int>(std::floor(start));
        int i_end = static_cast<int>(std::floor(end));

        x_inter(row, col) =
            image.block(row, i_start, 1, i_end - i_start + 1).mean();
      }
    }

    // Y interpolation
    // calculate corresponding original image coords
    auto scaled_rows = images_[l].rows();
    y.resize(scaled_rows);
    for (Eigen::Index i = 0; i < scaled_rows; ++i) {
      auto y_scaled = i * scale_factor_y;
      y[i] = y_scaled;
    }
    // interpolation
    for (Eigen::Index col = 0; col < scaled_cols; ++col) {
      for (Eigen::Index row = 0; row < scaled_rows; ++row) {
        // average range for single pixel
        float start = y[row];
        float end = (row + 1) < scaled_rows ? y[row + 1] : image.rows() - 1;

        int i_start = static_cast<int>(std::floor(start));
        int i_end = static_cast<int>(std::floor(end));

        images_[l](row, col) =
            x_inter.block(i_start, col, i_end - i_start + 1, 1).mean();
      }
    }
  }
}

void ScalePyramid::fill_bilinear_scale(const ImageFloat& image) {
  images_[0] = image;
  Eigen::ArrayXf x;
  Eigen::ArrayXi x1;
  Eigen::ArrayXi x2;
  Eigen::ArrayXf y;
  Eigen::ArrayXi y1;
  Eigen::ArrayXi y2;

  for (int l = 1; l < levels_; ++l) {
    auto scale_factor_x =
        1.f / (static_cast<float>(images_[l].cols()) / images_[l - 1].cols());
    auto scale_factor_y =
        1.f / (static_cast<float>(images_[l].rows()) / images_[l - 1].rows());

    x.resize(images_[l].cols());
    x1.resize(images_[l].cols());
    x2.resize(images_[l].cols());
    for (Eigen::Index i = 0; i < images_[l].cols(); ++i) {
      auto x_scaled = i * scale_factor_x;
      x[i] = x_scaled;
      x1[i] = std::min(
          static_cast<Eigen::Index>(floorf(x_scaled)),
          images_[l - 1].cols() - 1
      );
      x2[i] = std::min(
          static_cast<Eigen::Index>(ceilf(x_scaled)),
          images_[l - 1].cols() - 1
      );

      if (x1[i] == x2[i]) {
        x2[i] = x2[i] + 1;
      }
    }
    y.resize(images_[l].rows());
    y1.resize(images_[l].rows());
    y2.resize(images_[l].rows());
    for (Eigen::Index i = 0; i < images_[l].rows(); ++i) {
      auto y_scaled = i * scale_factor_y;
      y[i] = y_scaled;
      y1[i] = std::min(
          static_cast<Eigen::Index>(floorf(y_scaled)),
          images_[l - 1].rows() - 1
      );
      y2[i] = std::min(
          static_cast<Eigen::Index>(ceilf(y_scaled)),
          images_[l - 1].rows() - 1
      );
      if (y1[i] == y2[i]) {
        y2[i] = y2[i] + 1;
      }
    }
    auto q11 = images_[l - 1](y1, x1);
    auto q12 = images_[l - 1](y2, x1);
    auto q21 = images_[l - 1](y1, x2);
    auto q22 = images_[l - 1](y2, x2);

    // interpolation
    auto p1 = q11.array().rowwise() * (x2.cast<float>() - x).transpose() +
              q12.array().rowwise() * (x - x1.cast<float>()).transpose();

    auto p2 = q21.array().rowwise() * (x2.cast<float>() - x).transpose() +
              q22.array().rowwise() * (x - x1.cast<float>()).transpose();

    auto p = p1.array().colwise() * (y2.cast<float>() - y) +
             p2.array().colwise() * (y - y1.cast<float>());

    // save scaled image
    gaussian_blur(p, images_[l], Gaussian3);
  }
}

float ScalePyramid::scale_factor() const {
  return scale_factor_;
}

std::pair<int, int> ScalePyramid::coords_to_zero_level(int level, int x, int y)
    const {
  if (level == 0) {
    return {x, y};
  } else {
    auto scale_factor_x =
        1.f / (static_cast<float>(images_[level].cols()) / images_[0].cols());
    auto scale_factor_y =
        1.f / (static_cast<float>(images_[level].rows()) / images_[0].rows());
    return {
        static_cast<int>(x * scale_factor_x),
        static_cast<int>(y * scale_factor_y)};
  }
}

}  // namespace orb_features
