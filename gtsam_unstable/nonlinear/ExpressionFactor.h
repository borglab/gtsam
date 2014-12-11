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

  static const int Dim = traits::dimension<T>::value;

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
    DefaultChart<T> chart;
    if (H) {
      const T value = expression_.value(x, keys_, dims_, *H);
      return chart.local(measurement_, value);
    } else {
      const T value = expression_.value(x);
      return chart.local(measurement_, value);
    }
  }

  virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const {
    // Only linearize if the factor is active
    if (!active(x))
      return boost::shared_ptr<JacobianFactor>();

    // Create a writeable JacobianFactor in advance
    // In case noise model is constrained, we need to provide a noise model
    bool constrained = noiseModel_->isConstrained();
    boost::shared_ptr<JacobianFactor> factor(
        constrained ? new JacobianFactor(keys_, dims_, Dim,
            boost::static_pointer_cast<noiseModel::Constrained>(noiseModel_)->unit()) :
            new JacobianFactor(keys_, dims_, Dim));

    // Wrap keys and VerticalBlockMatrix into structure passed to expression_
    VerticalBlockMatrix& Ab = factor->matrixObject();
    JacobianMap jacobianMap(keys_, Ab);

    // Zero out Jacobian so we can simply add to it
    Ab.matrix().setZero();

    // Get value and Jacobians, writing directly into JacobianFactor
    T value = expression_.value(x, jacobianMap); // <<< Reverse AD happens here !

    // Evaluate error and set RHS vector b
    DefaultChart<T> chart;
    Ab(size()).col(0) = -chart.local(measurement_, value);

    // Whiten the corresponding system, Ab already contains RHS
    Vector dummy(Dim);
    noiseModel_->WhitenSystem(Ab.matrix(), dummy);

    return factor;
  }
};
// ExpressionFactor

/**
 *  ExpressionFactor Factories
 *  They obviate the need for making Factor classes that are almost empty.
 *  They also takes a default noise model.
 *  TODO: Ternary version
 */

/**
 *  A functor that creates unary expression factors on demand
 *  Example usage:
 *    MakeUnaryFactor<double, Event> MakePrior(&Event::height, model);
 *    ExpressionFactor<double> factor = MakePrior(z, eventExpr);
 */
template<typename T, typename A1>
class MakeUnaryFactor {

  typedef typename UnaryExpression<T, A1>::Method Method;
  typedef typename UnaryExpression<T, A1>::Function Function;

  Function function_;
  SharedNoiseModel model_;

public:

  /// Constructor with a binary function
  MakeUnaryFactor(Function function, const SharedNoiseModel& model) :
      function_(function), model_(model) {
  }

  /// Constructor with a unary method pointer
  MakeUnaryFactor(Method method, const SharedNoiseModel& model) :
      function_(boost::bind(method, _1, _2)), model_(model) {

  }

  /// Operator just needs noise model, measurement, and two expressions
  ExpressionFactor<T> operator()(double measurement,
      const Expression<A1>& expression1) {
    // Create expression and return factor
    Expression<T> expression(function_, expression1);
    return ExpressionFactor<T>(model_, measurement, expression);
  }

};

/**
 *  A functor that creates binary expression factors on demand
 *  Example usage:
 *    MakeBinaryFactor<double, Event, Point3> MakeFactor(&Event::toa, model);
 *    ExpressionFactor<double> factor = MakeFactor(z, eventExpr, microphoneExpr);
 */
template<typename T, typename A1, typename A2>
class MakeBinaryFactor {

  typedef typename BinaryExpression<T, A1, A2>::Method Method;
  typedef typename BinaryExpression<T, A1, A2>::Function Function;

  Function function_;
  SharedNoiseModel model_;

public:

  /// Constructor with a binary function
  MakeBinaryFactor(Function function, const SharedNoiseModel& model) :
      function_(function), model_(model) {
  }

  /// Constructor with a unary method pointer
  MakeBinaryFactor(Method method, const SharedNoiseModel& model) :
      function_(boost::bind(method, _1, _2, _3, _4)), model_(model) {

  }

  /// Operator just needs noise model, measurement, and two expressions
  ExpressionFactor<T> operator()(double measurement,
      const Expression<A1>& expression1, const Expression<A2>& expression2) {
    // Create expression and return factor
    Expression<T> expression(function_, expression1, expression2);
    return ExpressionFactor<T>(model_, measurement, expression);
  }

};

} // \ namespace gtsam

