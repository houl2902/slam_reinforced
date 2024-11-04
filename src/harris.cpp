#include "harris.h"

namespace orb_features {
void compute_harris_response(
    const ImageFloat& image,
    std::vector<FeaturePoint>& features,
    int block_size,
    float k
) {
  float scale = 1.f / ((1 << 2) * block_size * 255.f);
  float scale_sq_sq = scale * scale * scale * scale;
  auto r = block_size / 2;
  for (auto& feature : features) {
    auto x = feature.x;
    auto y = feature.y;

    if (x > r && y > r && y <= (image.rows() - r) && x <= (image.cols() - r)) {
      float a = 0, b = 0, c = 0;
      for (int by = 0; by < block_size; ++by) {
        for (int bx = 0; bx < block_size; ++bx) {
          // Sobel method, y-invert
          auto bxi = x + bx - r;
          auto byi = y + by - r;
          if ((bxi - 1) >= 0 && (byi - 1) >= 0 && (bxi + 1) < image.cols() &&
              (byi + 1) < image.rows()) {
            auto ix = (image(byi, bxi + 1) - image(byi, bxi - 1)) * 2 +
                      (image(byi - 1, bxi + 1) - image(byi - 1, bxi - 1)) +
                      (image(byi + 1, bxi + 1) - image(byi + 1, bxi - 1));

            auto iy = (image(byi + 1, bxi) - image(byi - 1, bxi)) * 2 +
                      (image(byi + 1, bxi - 1) - image(byi - 1, bxi - 1)) +
                      (image(byi + 1, bxi + 1) - image(byi - 1, bxi + 1));
            a += ix * ix;
            b += iy * iy;
            c += ix * iy;
          }
        }
      }
      feature.response = (a * b - c * c - k * (a + b) * (a + b)) * scale_sq_sq;
    }
  }
}
}  // namespace orb_features
