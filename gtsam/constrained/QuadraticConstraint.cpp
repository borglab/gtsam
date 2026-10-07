/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QuadraticConstraint.cpp
 * @brief   Quadratic constraint implementations.
 * @author  Frank Dellaert
 */

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/nonlinear/Values.h>

#include <stdexcept>

namespace gtsam {
namespace {

/* ************************************************************************* */
double SenseSign(QuadraticConstraint::Sense sense) {
  return sense == QuadraticConstraint::Sense::GreaterEqual ? -1.0 : 1.0;
}

/* ************************************************************************* */
Matrix VectorOrMatrixAsMatrix(const Values& values, Key key) {
  const Value& value = values.at(key);
  if (const auto* vectorValue =
          dynamic_cast<const GenericValue<Vector>*>(&value)) {
    return vectorValue->value();
  }
  if (const auto* matrixValue =
          dynamic_cast<const GenericValue<Matrix>*>(&value)) {
    return matrixValue->value();
  }
  throw std::invalid_argument(
      "QuadraticConstraint: only Vector and Matrix Values entries are "
      "supported.");
}

/* ************************************************************************* */
Vector ConstraintError(const QuadraticConstraint& constraint,
                       const Values& values, OptionalMatrixVecType H) {
  const Matrix X = VectorOrMatrixAsMatrix(values, constraint.key());
  if (X.rows() != constraint.A().rows()) {
    throw std::invalid_argument(
        "QuadraticConstraint: value dimension does not match A.");
  }

  const bool linear = constraint.hasLinearTerm();
  if (linear && X.cols() != 1) {
    throw std::invalid_argument(
        "QuadraticConstraint: a linear term requires a one-column value.");
  }

  const Matrix AX = constraint.A() * X;
  const double sign = SenseSign(constraint.sense());
  if (H) {
    Matrix gradient = sign * (constraint.A() + constraint.A().transpose()) * X;
    if (linear) gradient += 2.0 * sign * constraint.a();
    const Eigen::Map<const Vector> vectorized(gradient.data(), gradient.size());
    (*H)[0] = vectorized.transpose();
  }
  double value = (X.transpose() * AX).trace() - constraint.b();
  if (linear) value += 2.0 * constraint.a().dot(X.col(0));
  return Vector1(sign * value);
}

}  // namespace

/* ************************************************************************* */
QuadraticConstraint::QuadraticConstraint(Key key, const Matrix& A, double b,
                                         Sense sense, double sigma)
    : QuadraticConstraint(key, A, Vector(), b, sense, sigma) {}

/* ************************************************************************* */
QuadraticConstraint::QuadraticConstraint(Key key, const Matrix& A,
                                         const Vector& a, double b,
                                         Sense sense, double sigma)
    : key_(key),
      A_(A),
      a_(a.size() == 0 ? Vector(Vector::Zero(A.rows())) : a),
      b_(b),
      sense_(sense),
      sigma_(sigma) {
  if (A_.rows() != A_.cols()) {
    throw std::invalid_argument("QuadraticConstraint: A must be square.");
  }
  if (a_.size() != A_.rows()) {
    throw std::invalid_argument(
        "QuadraticConstraint: a must match the dimension of A.");
  }
  if (sigma_ <= 0.0) {
    throw std::invalid_argument("QuadraticConstraint: sigma must be positive.");
  }
}

/* ************************************************************************* */
NonlinearInequalityConstraint::shared_ptr
QuadraticConstraint::createInequalityFactor() const {
  if (isEquality()) {
    throw std::invalid_argument(
        "QuadraticConstraint: equality constraints cannot create inequality "
        "factors.");
  }
  return std::make_shared<QuadraticInequalityConstraintFactor>(*this);
}

/* ************************************************************************* */
Vector QuadraticEqualityConstraintFactor::unwhitenedError(
    const Values& values, OptionalMatrixVecType H) const {
  return ConstraintError(constraint_, values, H);
}

/* ************************************************************************* */
Vector QuadraticInequalityConstraintFactor::unwhitenedExpr(
    const Values& values, OptionalMatrixVecType H) const {
  return ConstraintError(constraint_, values, H);
}

}  // namespace gtsam
