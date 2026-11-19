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

class AugmentedLagrangianParams;

/// Solver choice for QpProblem::optimize convenience methods.
enum class QpSolverType {
  Sparse,              ///< Sparse working-factor-graph active-set QP mode.
  Dense,               ///< Dense active-set Schur-complement QP mode.
  AugmentedLagrangian  ///< Gaussian-factor ALM with native policy defaults.
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

  /** Optimize from finite initial values using the selected numerical backend
   * and Automatic formulation. Dual permits an infeasible proximal center;
   * a Primal fallback requires an inequality-feasible point. ALM uses native
   * defaults and accepts infeasible starts, preserving supplied value shapes.
   */
  Values optimize(const Values& initialValues,
                  QpSolverType solverType = QpSolverType::Sparse) const;

  /** Optimize using the selected numerical backend and Automatic formulation.
   * Dual uses a zero proximal center; Primal first finds a feasible point. ALM
   * uses zero vectors and its native defaults; inspect the direct solver state
   * when an explicit convergence certificate is needed. */
  Values optimize(QpSolverType solverType = QpSolverType::Sparse) const;

  /** Optimize with explicitly supplied Gaussian ALM options. The parameter
   * object is copied. In particular, lmParams.multifrontalParams.qrMode can
   * disable leaf QR without changing the ALM policy or objective.
   */
  Values optimize(const Values& initialValues,
                  const AugmentedLagrangianParams& params) const;

  /** Optimize from zero vectors using explicitly supplied Gaussian ALM options.
   * Use the direct solver when convergence state or multipliers are needed.
   */
  Values optimize(const AugmentedLagrangianParams& params) const;
};

}  // namespace gtsam
