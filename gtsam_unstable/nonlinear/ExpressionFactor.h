/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Expressions for Block Automatic Differentiation
 */

#include <gtsam_unstable/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>

namespace gtsam {

/**
 * Factor that supports arbitrary expressions via AD
 */
template<class T>
class ExpressionFactor: public NoiseModelFactor {

  const T measurement_;
  const Expression<T> expression_;

public:

  /// Constructor
  ExpressionFactor(const SharedNoiseModel& noiseModel, //
      const T& measurement, const Expression<T>& expression) :
      NoiseModelFactor(noiseModel, expression.keys()), //
      measurement_(measurement), expression_(expression) {
    if (!noiseModel)
      throw std::invalid_argument("ExpressionFactor: no NoiseModel.");
    if (noiseModel->dim() != T::dimension)
      throw std::invalid_argument(
          "ExpressionFactor was created with a NoiseModel of incorrect dimension.");
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    if (H) {
      assert(H->size()==size());
      JacobianMap jacobians;
      T value = expression_.value(x, jacobians);
      // move terms to H, which is pre-allocated to correct size
      move(jacobians, *H);
      return measurement_.localCoordinates(value);
    } else {
      const T& value = expression_.value(x);
      return measurement_.localCoordinates(value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {

    using namespace boost::adaptors;

    // Only linearize if the factor is active
    if (!this->active(x))
      return boost::shared_ptr<JacobianFactor>();

    // Get dimensions of matrices
    std::map<Key, size_t> map = expression_.dimensions();
    size_t n = map.size();

    // Get actual dimensions. TODO: why can't we pass map | map_values directly?
    std::vector<size_t> dims(n);
    boost::copy(map | map_values, dims.begin());

    // Construct block matrix, is of right size but un-initialized
    VerticalBlockMatrix Ab(dims, T::dimension, true);

    // Create and zero out blocks to be passed to expression_
    DenseIndex i = 0; // For block index
    typedef std::pair<Key, size_t> Pair;
    BOOST_FOREACH(const Pair& keyValue, map) {
      Ab(i++) = zeros(T::dimension, keyValue.second);
    }

    // Evaluate error to get Jacobians and RHS vector b
    // JacobianMap terms;
    T value = expression_.value(x);
    Vector b = -measurement_.localCoordinates(value);

    // Whiten the corresponding system now
    // TODO ! this->noiseModel_->WhitenSystem(A, b);

    // Fill in RHS
    Ab(n).col(0) = b;

    // TODO pass unwhitened + noise model to Gaussian factor
    // For now, only linearized constrained factors have noise model at linear level!!!
    noiseModel::Constrained::shared_ptr constrained = //
        boost::dynamic_pointer_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained) {
      return boost::make_shared<JacobianFactor>(map | map_keys, Ab,
          constrained->unit());
    } else
      return boost::make_shared<JacobianFactor>(map | map_keys, Ab);
  }
};
// ExpressionFactor

}

