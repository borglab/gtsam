/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.cpp
 * @brief   A factor with a quadratic error function - a Gaussian
 * @brief   GaussianFactor
 * @author  Fan Jiang
 */

// \callgraph

#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

double GaussianFactor::error(const VectorValues& c) const {
  throw std::runtime_error("GaussianFactor::error is not implemented");
}

double GaussianFactor::error(const HybridValues& c) const {
  return this->error(c.continuous());
}

VectorValues GaussianFactor::hessianDiagonal() const {
  VectorValues d;
  hessianDiagonalAdd(d);
  return d;
}

}  // namespace gtsam
