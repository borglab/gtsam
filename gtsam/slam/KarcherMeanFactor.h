/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file KarcherMeanFactor.h
 * @author Frank Dellaert
 * @date March 2019
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/JacobianFactor.h>

#include <map>
#include <vector>
#include <optional>

namespace gtsam {
/**
 * Optimize for the Karcher mean, minimizing the geodesic distance to each of
 * the given rotations, by constructing a factor graph out of simple
 * PriorFactors.
 */
template <class T>
T FindKarcherMean(const std::vector<T, Eigen::aligned_allocator<T>> &rotations);

template <class T> T FindKarcherMean(std::initializer_list<T> &&rotations);

/**
 * The KarcherMeanFactor creates a constraint on all SO(n) variables with
 * given keys that the Karcher mean (see above) will stay the same. Note the
 * mean itself is irrelevant to the constraint and is not a parameter: the
 * constraint is implemented as enforcing that the sum of local updates is
 * equal to zero, hence creating a rank-dim constraint. Note it is implemented
 * as a soft constraint, as typically it is used to fix a gauge freedom.
 * */
template <class T> class KarcherMeanFactor : public NonlinearFactor {
  // Compile time dimension: can be -1
  enum { D = traits<T>::dimension };

  // Runtime dimension: always >=0
  size_t d_;

  /// Constant Jacobian made of d*d identity matrices
  std::shared_ptr<JacobianFactor> whitenedJacobian_;

public:
  /**
   * Construct from given keys.
   * If parameter beta is given, it acts as a precision = 1/sigma^2, and
   * the Jacobian will be multiplied with 1/sigma = sqrt(beta).
   */
  template <typename CONTAINER>
  KarcherMeanFactor(const CONTAINER &keys, int d = D,
                    std::optional<double> beta = {});

  /// Destructor
  ~KarcherMeanFactor() override {}

  /// Calculate the error of the factor: always zero
  double error(const Values &c) const override { return 0; }

  /// get the dimension of the factor (number of rows on linearization)
  size_t dim() const override { return d_; }

  /// linearize to a GaussianFactor
  std::shared_ptr<GaussianFactor> linearize(const Values &c) const override {
    return whitenedJacobian_;
  }
};
// \KarcherMeanFactor
} // namespace gtsam
