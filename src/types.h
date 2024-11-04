#pragma once
#include <Eigen/Dense>

namespace orb_features {
using Image =
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ImageFloat =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using IndexArray = Eigen::Array<Eigen::Index, -1, 1, 0>;
}  // namespace orb_features
