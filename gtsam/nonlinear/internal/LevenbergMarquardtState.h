/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LevenbergMarquardtState.h
 * @brief   A LevenbergMarquardtState class containing most of the logic for Levenberg-Marquardt
 * @author  Frank Dellaert
 * @date    April 2016
 */

#include "NonlinearOptimizerState.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gtsam {

namespace internal {

// TODO(frank): once Values supports move, we can make State completely functional.
// As it is now, increaseLambda is non-const or otherwise we make a Values copy every time
// decreaseLambda would also benefit from a working Values move constructor
struct LevenbergMarquardtState : public NonlinearOptimizerState {
  typedef LevenbergMarquardtState This;

  // TODO(frank): make these const once increaseLambda can be made const
  double lambda;
  double currentFactor;
  int totalNumberInnerIterations;  ///< The total number of inner iterations in the
                                   // optimization (for each iteration, LM tries multiple
                                   // inner iterations with different lambdas)

  LevenbergMarquardtState(const Values& initialValues, double error, double lambda,
                          double currentFactor, unsigned int iterations = 0,
                          unsigned int totalNumberInnerIterations = 0)
      : NonlinearOptimizerState(initialValues, error, iterations),
        lambda(lambda),
        currentFactor(currentFactor),
        totalNumberInnerIterations(totalNumberInnerIterations) {}

  // Constructor version that takes ownership of values
  LevenbergMarquardtState(Values&& initialValues, double error, double lambda, double currentFactor,
                          unsigned int iterations = 0, unsigned int totalNumberInnerIterations = 0)
      : NonlinearOptimizerState(initialValues, error, iterations),
        lambda(lambda),
        currentFactor(currentFactor),
        totalNumberInnerIterations(totalNumberInnerIterations) {}

  // Applies policy to *increase* lambda: should be used if the current update was NOT successful
  // TODO(frank): non-const method until Values properly support std::move
  void increaseLambda(const LevenbergMarquardtParams& params) {
    lambda *= currentFactor;
    totalNumberInnerIterations += 1;
    if (!params.useFixedLambdaFactor) {
      currentFactor *= 2.0;
    }
  }

  // Apply policy to decrease lambda if the current update was successful
  // stepQuality not used in the naive policy)
  // Take ownsership of newValues, must be passed an rvalue
  std::unique_ptr<This> decreaseLambda(const LevenbergMarquardtParams& params, double stepQuality,
                                       Values&& newValues, double newError) const {
    double newLambda = lambda, newFactor = currentFactor;
    if (params.useFixedLambdaFactor) {
      newLambda /= currentFactor;
    } else {
      // TODO(frank): odd that currentFactor is not used to change lambda here...
      newLambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * stepQuality - 1.0, 3));
      newFactor = 2.0 * currentFactor;
    }
    newLambda = std::max(params.lambdaLowerBound, newLambda);
    return std::unique_ptr<This>(new This(std::move(newValues), newError, newLambda, newFactor,
                                          iterations + 1, totalNumberInnerIterations + 1));
  }

  /** Small struct to cache objects needed for damping.
   * This is used in buildDampedSystem  */
  struct CachedModel {
    CachedModel() {}  // default int makes zero-size matrices
    CachedModel(int dim, double sigma)
        : A(Matrix::Identity(dim, dim)),
          b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    CachedModel(int dim, double sigma, const Vector& diagonal)
        : A(Eigen::DiagonalMatrix<double, Eigen::Dynamic>(diagonal)),
          b(Vector::Zero(dim)),
          model(noiseModel::Isotropic::Sigma(dim, sigma)) {}
    Matrix A;
    Vector b;
    SharedDiagonal model;
  };

  // Small cache of A|b|model indexed by dimension. Can save many mallocs.
  mutable std::vector<CachedModel> noiseModelCache;
  CachedModel* getCachedModel(size_t dim) const {
    if (dim >= noiseModelCache.size())
      noiseModelCache.resize(dim+1);
    CachedModel* item = &noiseModelCache[dim];
    if (!item->model)
      *item = CachedModel(dim, 1.0 / std::sqrt(lambda));
    return item;
  };

  /// Build a damped system for a specific lambda, vanilla version
  GaussianFactorGraph buildDampedSystem(GaussianFactorGraph damped /* gets copied */) const {
    noiseModelCache.resize(0);
    // for each of the variables, add a prior
    damped.reserve(damped.size() + values.size());
    std::map<Key,size_t> dims = values.dims();
    for (const auto& key_dim : dims) {
      const Key& key = key_dim.first;
      const size_t& dim = key_dim.second;
      const CachedModel* item = getCachedModel(dim);
      damped += boost::make_shared<JacobianFactor>(key, item->A, item->b, item->model);
    }
    return damped;
  }

  /// Build a damped system, use hessianDiagonal per variable (more expensive)
  GaussianFactorGraph buildDampedSystem(GaussianFactorGraph damped,  // gets copied
                                        const VectorValues& sqrtHessianDiagonal) const {
    noiseModelCache.resize(0);
    damped.reserve(damped.size() + values.size());
    for (const auto& key_vector : sqrtHessianDiagonal) {
      try {
        const Key key = key_vector.first;
        const size_t dim = key_vector.second.size();
        CachedModel* item = getCachedModel(dim);
        item->A.diagonal() = sqrtHessianDiagonal.at(key);  // use diag(hessian)
        damped += boost::make_shared<JacobianFactor>(key, item->A, item->b, item->model);
      } catch (const std::out_of_range&) {
        continue;  // Don't attempt any damping if no key found in diagonal
      }
    }
    return damped;
  }
};

}  // namespace internal

} /* namespace gtsam */
