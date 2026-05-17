/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpProblem.h
 * @brief   Quadratic programming problem over Vector and Matrix values.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/linear/GaussianFactor.h>

#include <memory>

namespace gtsam {

/// Solver choice for QpProblem::optimize convenience methods.
enum class QpSolverType {
  Sparse,  ///< Sparse working-factor-graph active-set QP mode.
  Dense    ///< Dense active-set Schur-complement QP mode.
};

/**
 * QP problem with affine quadratic costs and linear constraints over direct
 * Vector and Matrix Values entries.
 */
class GTSAM_EXPORT QpProblem : public ConstrainedOptProblem {
 public:
  using Base = ConstrainedOptProblem;
  using This = QpProblem;
  using shared_ptr = std::shared_ptr<This>;

  /** Default constructor for I/O. */
  QpProblem() = default;

  /** Add an affine quadratic cost. */
  void addCost(const QpCost& cost) { costs_.emplace_shared<QpCost>(cost); }

  /** Add an affine quadratic cost from a Hessian factor. */
  void addCost(const HessianFactor& factor) { addCost(QpCost(factor)); }

  /** Add an affine quadratic cost from any Gaussian factor. */
  void addCost(const GaussianFactor& factor) { addCost(QpCost(factor)); }

  /** Add an affine quadratic cost from a shared Gaussian factor. */
  void addCost(const GaussianFactor::shared_ptr& factor) { addCost(*factor); }

  /** Add a linear constraint. */
  void addConstraint(const LinearConstraint& constraint);

  /** Optimize from a feasible initial point using the selected QP solver. */
  Values optimize(const Values& initialValues,
                  QpSolverType solverType = QpSolverType::Sparse) const;

  /** Find a vector-valued feasible point and optimize using the selected
   * solver. */
  Values optimize(QpSolverType solverType = QpSolverType::Sparse) const;
};

}  // namespace gtsam
