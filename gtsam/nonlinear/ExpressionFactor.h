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

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <numeric>

namespace gtsam {

/**
 * Factor that supports arbitrary expressions via AD
 */
template<class T>
class ExpressionFactor: public NoiseModelFactor {

protected:

  T measurement_; ///< the measurement to be compared with the expression
  Expression<T> expression_; ///< the expression that is AD enabled
  FastVector<int> dims_; ///< dimensions of the Jacobian matrices

  static const int Dim = traits<T>::dimension;

public:

  /// Constructor
  ExpressionFactor(const SharedNoiseModel& noiseModel, //
      const T& measurement, const Expression<T>& expression) :
      measurement_(measurement), expression_(expression) {
    if (!noiseModel)
      throw std::invalid_argument("ExpressionFactor: no NoiseModel.");
    if (noiseModel->dim() != Dim)
      throw std::invalid_argument(
          "ExpressionFactor was created with a NoiseModel of incorrect dimension.");
    noiseModel_ = noiseModel;

    // Get keys and dimensions for Jacobian matrices
    // An Expression is assumed unmutable, so we do this now
    boost::tie(keys_, dims_) = expression_.keysAndDims();
  }

  /**
   * Error function *without* the NoiseModel, \f$ h(x)-z \f$.
   * We override this method to provide
   * both the function evaluation and its derivative(s) in H.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
     if (H) {
      const T hx = expression_.value(x, keys_, dims_, *H); // h(x)
      return traits<T>::Local(measurement_, hx); // h(x) - z
    } else {
      const T hx = expression_.value(x); // h(x)
      return traits<T>::Local(measurement_, hx); // h(x) - z
    }
  }

  /**
   * Linearize the factor into a JacobianFactor
   */
  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x_bar) const {
    // Only linearize if the factor is active
    if (!active(x_bar))
      return boost::shared_ptr<JacobianFactor>();

    // Create a writable JacobianFactor in advance
    // In case noise model is constrained, we need to provide a noise model
    bool constrained = noiseModel_->isConstrained();
    boost::shared_ptr<JacobianFactor> factor(
        constrained ? new JacobianFactor(keys_, dims_, Dim,
            boost::static_pointer_cast<noiseModel::Constrained>(noiseModel_)->unit()) :
            new JacobianFactor(keys_, dims_, Dim));

    // Wrap keys and VerticalBlockMatrix into structure passed to expression_
    VerticalBlockMatrix& Ab = factor->matrixObject(); // reference, no malloc !
    JacobianMap jacobianMap(keys_, Ab);

    // Zero out Jacobian so we can simply add to it
    Ab.matrix().setZero();

    // Get value and Jacobians, writing directly into JacobianFactor
    T hx = expression_.value(x_bar, jacobianMap); // <<< Reverse AD happens here !

    // Evaluate error and set RHS vector b = - (h(x_bar) - z) = z-h(x_bar)
    // Indeed, nonlinear error |h(x_bar+dx)-z| ~ |h(x_bar) + A*dx - z|
    //                                         = |A*dx - (z-h(x_bar))|
    Ab(size()).col(0) = - traits<T>::Local(measurement_, hx); // - unwhitenedError(x_bar)

    // Whiten the corresponding system, Ab already contains RHS
    Vector dummy(Dim);
    noiseModel_->WhitenSystem(Ab.matrix(), dummy);

    return factor;
  }
};
// ExpressionFactor

} // \ namespace gtsam

