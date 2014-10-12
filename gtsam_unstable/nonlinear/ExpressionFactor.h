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
      Augmented<T> augmented = expression_.augmented(x);
      // move terms to H, which is pre-allocated to correct size
      augmented.move(*H);
      return measurement_.localCoordinates(augmented.value());
    } else {
      const T& value = expression_.value(x);
      return measurement_.localCoordinates(value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {

    // Only linearize if the factor is active
    if (!this->active(x))
      return boost::shared_ptr<JacobianFactor>();

    // Evaluate error to get Jacobians and RHS vector b
    Augmented<T> augmented = expression_.augmented(x);
    Vector b = -measurement_.localCoordinates(augmented.value());
    // check(noiseModel_, b); // TODO: use, but defined in NonlinearFactor.cpp

    // Whiten the corresponding system now
    // TODO ! this->noiseModel_->WhitenSystem(A, b);

    // Terms, needed to create JacobianFactor below, are already here!
    const JacobianMap& terms = augmented.jacobians();
    size_t n = terms.size();

    // Get dimensions of matrices
    std::vector<size_t> dimensions;
    dimensions.reserve(n);
    std::vector<Key> keys;
    keys.reserve(n);
    for (JacobianMap::const_iterator it = terms.begin(); it != terms.end();
        ++it) {
      const std::pair<Key, Matrix>& term = *it;
      Key key = term.first;
      const Matrix& Ai = term.second;
      dimensions.push_back(Ai.cols());
      keys.push_back(key);
    }

    // Construct block matrix
    VerticalBlockMatrix Ab(dimensions, b.size(), true);

    // Check and add terms
    DenseIndex i = 0; // For block index
    for (JacobianMap::const_iterator it = terms.begin(); it != terms.end();
        ++it) {
      const std::pair<Key, Matrix>& term = *it;
      const Matrix& Ai = term.second;
      Ab(i++) = Ai;
    }

    Ab(n).col(0) = b;

    // TODO pass unwhitened + noise model to Gaussian factor
    // For now, only linearized constrained factors have noise model at linear level!!!
    noiseModel::Constrained::shared_ptr constrained = //
        boost::dynamic_pointer_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained) {
      return boost::make_shared<JacobianFactor>(keys, Ab, constrained->unit());
    } else
      return boost::make_shared<JacobianFactor>(keys, Ab);
  }
};
// ExpressionFactor

}

