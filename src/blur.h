#pragma once
#include "types.h"

namespace orb_features {
enum KernelSize {
  Gaussian3,
  Gaussian5,
};
void gaussian_blur(
    const ImageFloat& src,
    ImageFloat& dst,
    KernelSize kernel_size
);
}  // namespace orb_features
