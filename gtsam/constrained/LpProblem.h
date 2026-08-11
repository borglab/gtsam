/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LpProblem.h
 * @brief   Linear programming problem over Vector and Matrix values.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace gtsam {

class ActiveSetSolverParams;

/**
 * Linear objective term over direct Vector and Matrix Values entries.
 *
 * A LpCost stores one Jacobian factor A*x-b and interprets the scalar objective
 * contribution as the sum of its rows. Matrix entries are interpreted by
 * column-major vectorization.
 */
class GTSAM_EXPORT LpCost {
 public:
  using shared_ptr = std::shared_ptr<LpCost>;

  /** Construct from a Jacobian factor. */
  explicit LpCost(const JacobianFactor& factor)
      : factor_(std::make_shared<JacobianFactor>(factor)) {}

  /// Return the stored Jacobian factor.
  const JacobianFactor& factor() const { return *factor_; }

  /** Evaluate this linear objective contribution. */
  double value(const Values& values) const;

 private:
  static Vector vectorOrMatrixValue(const Values& values, Key key);
  static VectorValues directVectorValues(const Values& values,
                                         const JacobianFactor& factor);

  JacobianFactor::shared_ptr factor_;
};

/**
 * LP problem with linear costs and constraints over direct Vector and Matrix
 * Values entries.
 */
class GTSAM_EXPORT LpProblem : public ConstrainedOptProblem {
 public:
  using Base = ConstrainedOptProblem;
  using This = LpProblem;
  using shared_ptr = std::shared_ptr<This>;

  /** Default constructor for I/O. */
  LpProblem() = default;

  /** Add a linear objective term. */
  void addCost(const LpCost& cost) { linearCosts_.push_back(cost); }

  /** Add a linear objective term from a Jacobian factor. */
  void addCost(const JacobianFactor& factor) { addCost(LpCost(factor)); }

  /** Add a linear constraint. */
  void addConstraint(const LinearConstraint& constraint);

  /// Return stored linear objective terms.
  const std::vector<LpCost>& linearCosts() const { return linearCosts_; }

  /** Evaluate the scalar LP objective. */
  double objective(const Values& values) const;

  /** Optimize from an initial point using the active-set solver. */
  Values optimize(
      const Values& initialValues,
      std::shared_ptr<ActiveSetSolverParams> params = nullptr) const;

  /** Find a vector-valued feasible point and optimize. */
  Values optimize(
      std::shared_ptr<ActiveSetSolverParams> params = nullptr) const;

 private:
  std::vector<LpCost> linearCosts_;
};

/* ************************************************************************* */
inline Vector LpCost::vectorOrMatrixValue(const Values& values, Key key) {
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
      "LpCost: only Vector and Matrix Values entries are supported.");
}

/* ************************************************************************* */
inline VectorValues LpCost::directVectorValues(const Values& values,
                                               const JacobianFactor& factor) {
  VectorValues result;
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    const Vector vector = vectorOrMatrixValue(values, *it);
    if (factor.getDim(it) != vector.size()) {
      throw std::invalid_argument(
          "LpCost: Jacobian block dimension does not match value dimension.");
    }
    result.insert(*it, vector);
  }
  return result;
}

/* ************************************************************************* */
inline double LpCost::value(const Values& values) const {
  return factor_->unweighted_error(directVectorValues(values, *factor_)).sum();
}

}  // namespace gtsam
