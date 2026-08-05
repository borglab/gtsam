/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearConstraint.cpp
 * @brief   Linear constraints for LP, QP, and QCQP problems.
 * @author  Frank Dellaert
 */

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/LinearConstraint.h>

#include <stdexcept>

namespace gtsam {
namespace {

/* ************************************************************************* */
Vector VectorOrMatrixValue(const Values& values, Key key) {
  const Value& value = values.at(key);
  if (const auto* vectorValue =
          dynamic_cast<const GenericValue<Vector>*>(&value)) {
    return vectorValue->value();
  }
  if (const auto* matrixValue =
          dynamic_cast<const GenericValue<Matrix>*>(&value)) {
    const Matrix& matrix = matrixValue->value();
    return Eigen::Map<const Vector>(matrix.data(), matrix.size());
  }
  throw std::invalid_argument(
      "LinearConstraint: only Vector and Matrix Values entries are supported.");
}

/* ************************************************************************* */
VectorValues DirectVectorValues(const Values& values,
                                const JacobianFactor& factor) {
  VectorValues result;
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    const Vector vector = VectorOrMatrixValue(values, *it);
    if (factor.getDim(it) != vector.size()) {
      throw std::invalid_argument(
          "LinearConstraint: Jacobian block dimension does not match value "
          "dimension.");
    }
    result.insert(*it, vector);
  }
  return result;
}

/* ************************************************************************* */
double SenseSign(LinearConstraint::Sense sense) {
  return sense == LinearConstraint::Sense::GreaterEqual ? -1.0 : 1.0;
}

/* ************************************************************************* */
Vector ConstraintError(const JacobianFactor& factor, const Values& values,
                       double sign) {
  return sign * factor.unweighted_error(DirectVectorValues(values, factor));
}

/* ************************************************************************* */
void FillJacobians(const JacobianFactor& factor, OptionalMatrixVecType H,
                   double sign) {
  if (!H) {
    return;
  }
  for (size_t i = 0; i < factor.size(); ++i) {
    (*H)[i] = sign * factor.getA(factor.begin() + i);
  }
}

}  // namespace

/* ************************************************************************* */
LinearConstraint::LinearConstraint(const JacobianFactor& factor, Sense sense,
                                   const Vector& sigmas)
    : factor_(std::make_shared<JacobianFactor>(factor)),
      sense_(sense),
      sigmas_(sigmas) {
  if (sigmas_.size() != factor.getb().size()) {
    throw std::invalid_argument(
        "LinearConstraint: sigma dimension must match constraint dimension.");
  }
}

/* ************************************************************************* */
NonlinearInequalityConstraint::shared_ptr
LinearConstraint::createInequalityFactor() const {
  if (isEquality()) {
    throw std::invalid_argument(
        "LinearConstraint: equality constraints cannot create inequality "
        "factors.");
  }
  return std::make_shared<LinearInequalityConstraintFactor>(*this);
}

/* ************************************************************************* */
Vector LinearEqualityConstraintFactor::unwhitenedError(
    const Values& values, OptionalMatrixVecType H) const {
  FillJacobians(constraint_.factor(), H, SenseSign(constraint_.sense()));
  return ConstraintError(constraint_.factor(), values,
                         SenseSign(constraint_.sense()));
}

/* ************************************************************************* */
Vector LinearInequalityConstraintFactor::unwhitenedExpr(
    const Values& values, OptionalMatrixVecType H) const {
  FillJacobians(constraint_.factor(), H, SenseSign(constraint_.sense()));
  return ConstraintError(constraint_.factor(), values,
                         SenseSign(constraint_.sense()));
}

}  // namespace gtsam
