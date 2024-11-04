#include "blur.h"
#include <unsupported/Eigen/CXX11/Tensor>

namespace orb_features {
namespace {
using TensorFloat = Eigen::Tensor<float, 2, Eigen::RowMajor>;

const TensorFloat gaussian_kernel_5 = [] {
  TensorFloat k(5, 5);
  k.setValues(
      {{1, 4, 6, 4, 1},
       {4, 16, 24, 16, 4},
       {6, 24, 36, 24, 6},
       {4, 16, 24, 16, 4},
       {1, 4, 6, 4, 1}}
  );
  k = k / 256.f;
  return k;
}();

const TensorFloat gaussian_kernel_3 = [] {
  TensorFloat k(3, 3);
  k.setValues({
      {1, 2, 1},
      {2, 4, 2},
      {1, 2, 1},
  });
  k = k / 16.f;
  return k;
}();

const std::array<std::pair<int, int>, 2> paddings_5{
    std::make_pair(
        gaussian_kernel_5.dimension(0) / 2,
        gaussian_kernel_5.dimension(0) / 2
    ),
    std::make_pair(
        gaussian_kernel_5.dimension(1) / 2,
        gaussian_kernel_5.dimension(1) / 2
    )};

const std::array<std::pair<int, int>, 2> paddings_3{
    std::make_pair(
        gaussian_kernel_3.dimension(0) / 2,
        gaussian_kernel_3.dimension(0) / 2
    ),
    std::make_pair(
        gaussian_kernel_3.dimension(1) / 2,
        gaussian_kernel_3.dimension(1) / 2
    )};
const Eigen::array<ptrdiff_t, 2> dims({0, 1});
}  // namespace

void gaussian_blur(
    const ImageFloat& src,
    ImageFloat& dst,
    KernelSize kernel_size
) {
  Eigen::TensorMap<TensorFloat> dst_tensor(
      dst.data(),
      {dst.rows(), dst.cols()}
  );
  Eigen::TensorMap<const TensorFloat> src_tensor(
      src.data(),
      {src.rows(), src.cols()}
  );
  switch (kernel_size) {
    case Gaussian3:
      dst_tensor = src_tensor.pad(paddings_3).convolve(gaussian_kernel_3, dims);
      break;
    case Gaussian5:
      dst_tensor = src_tensor.pad(paddings_5).convolve(gaussian_kernel_5, dims);
      break;
  }
}
}  // namespace orb_features
