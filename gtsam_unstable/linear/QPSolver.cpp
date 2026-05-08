/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSolver.cpp
 * @brief    Implementation of QPSolver.
 * @author   Frank Dellaert
 * @date     May 2026
 */

#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <gtsam_unstable/linear/QPInitSolver.h>

namespace gtsam {

// =============================================================================
// Constructor: factor H once, build key layout
// =============================================================================

QPSolver::QPSolver(const QP& qp) : problem_(qp) {
  // Variable indices for dual computation.
  equalityVariableIndex_ = VariableIndex(qp.equalities);
  inequalityVariableIndex_ = VariableIndex(qp.inequalities);
  constrainedKeys_ = qp.equalities.keys();
  constrainedKeys_.merge(qp.inequalities.keys());

  // Build an ordering over the COST variables only.  Using the full-problem
  // ordering (cost + constraints) can introduce fill-in patterns that make the
  // cost-only Cholesky appear ill-conditioned for structured problems (e.g. MPC
  // with causal dynamics).  A cost-only COLAMD ordering avoids this.
  hessianOrdering_ = Ordering::Colamd(VariableIndex(qp.cost));

  // Eliminate the cost graph once to get a sparse Cholesky (GaussianBayesNet).
  costBN_ = qp.cost.eliminateSequential(hessianOrdering_);

  // Use the unconstrained optimum to discover key dimensions and cache H^{-1}η.
  VectorValues x0 = costBN_->optimize();
  size_t offset = 0;
  for (const Key key : hessianOrdering_) {
    if (x0.exists(key)) {
      keyDims_[key] = static_cast<size_t>(x0.at(key).size());
      keyOffsets_[key] = offset;
      offset += keyDims_[key];
    }
  }
  totalDim_ = offset;

  // Cache H^{-1}η (unconstrained optimum) in dense layout.
  hinvEtaVec_ = toVector(x0);
}

// =============================================================================
// Dense layout helpers
// =============================================================================

Vector QPSolver::toVector(const VectorValues& vv) const {
  Vector v = Vector::Zero(totalDim_);
  for (const auto& [key, vec] : vv) {
    auto it = keyOffsets_.find(key);
    if (it != keyOffsets_.end()) v.segment(it->second, vec.size()) = vec;
  }
  return v;
}

VectorValues QPSolver::toValues(const Vector& v) const {
  VectorValues vv;
  for (const auto& [key, dim] : keyDims_) {
    size_t off = keyOffsets_.at(key);
    vv.insert(key, v.segment(off, dim));
  }
  return vv;
}

Vector QPSolver::applyHinv(const Vector& v) const {
  VectorValues vv = toValues(v);
  return toVector(
      costBN_->backSubstitute(costBN_->backSubstituteTranspose(vv)));
}

void QPSolver::fillConstraintRow(const JacobianFactor& factor, size_t row,
                                     Matrix& A_W, Vector& b_W) const {
  size_t nRows = factor.rows();
  // Fill A columns for each variable in the factor.
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    Key key = *it;
    auto offIt = keyOffsets_.find(key);
    if (offIt != keyOffsets_.end()) {
      A_W.block(row, offIt->second, nRows, keyDims_.at(key)) =
          factor.getA(it);
    }
  }
  // Fill b.
  b_W.segment(row, nRows) = factor.getb();
}

size_t QPSolver::countConstraintRows(
    const InequalityFactorGraph& workingSet) const {
  size_t m = 0;
  for (const LinearEquality::shared_ptr& f : problem_.equalities) m += f->rows();
  for (const LinearInequality::shared_ptr& f : workingSet)
    if (f->active()) m += f->rows();
  return m;
}

// =============================================================================
// KKT solve via Schur complement
// =============================================================================

std::pair<VectorValues, VectorValues> QPSolver::solveKKT(
    const InequalityFactorGraph& workingSet) const {
  const size_t m = countConstraintRows(workingSet);

  if (m == 0) {
    // No active constraints: return cached unconstrained optimum.
    return {toValues(hinvEtaVec_), VectorValues()};
  }

  // Build dense A_W (m×n) and b_W (m×1).
  Matrix A_W = Matrix::Zero(m, totalDim_);
  Vector b_W = Vector::Zero(m);
  // Also track dual keys per row (for packing the λ VectorValues).
  std::vector<Key> rowDualKey(m);
  std::vector<size_t> rowDim(m, 1);  // currently all constraints are 1-D

  size_t row = 0;
  for (const LinearEquality::shared_ptr& f : problem_.equalities) {
    fillConstraintRow(*f, row, A_W, b_W);
    for (size_t r = 0; r < f->rows(); ++r) rowDualKey[row + r] = f->dualKey();
    row += f->rows();
  }
  for (const LinearInequality::shared_ptr& f : workingSet) {
    if (!f->active()) continue;
    fillConstraintRow(*f, row, A_W, b_W);
    for (size_t r = 0; r < f->rows(); ++r) rowDualKey[row + r] = f->dualKey();
    row += f->rows();
  }

  // KKT system (stationarity + primal feasibility):
  //   H x = η + A_W' λ    (stationarity: ∇f(x) = A_W' λ)
  //   A_W x = b_W          (primal feasibility)
  //
  // Solving by Schur complement:
  //   x = H^{-1}(η + A_W' λ)
  //   Substituting: A_W H^{-1}(η + A_W' λ) = b_W
  //   S_W λ = b_W - A_W H^{-1} η     where S_W = A_W H^{-1} A_W'
  //
  // Sign convention matches buildDualGraph: A_W' λ = ∇f(x) = Hx - η,
  // so λ > 0 for a constraint whose removal improves the objective
  // (consistent with identifyLeavingConstraint).

  // Apply H^{-1} to each row of A_W (= column of A_W^T) via sparse back-substitution.
  Matrix HinvAwT(totalDim_, m);
  for (size_t j = 0; j < m; ++j)
    HinvAwT.col(j) = applyHinv(A_W.row(j).transpose());

  // S_W = A_W * H^{-1} A_W^T  (m×m)
  Matrix S_W = A_W * HinvAwT;

  // λ = S_W^{-1}(b_W - A_W * H^{-1} η)
  Eigen::LDLT<Matrix> S_ldlt(S_W);
  Vector lambda = S_ldlt.solve(b_W - A_W * hinvEtaVec_);

  // x = H^{-1}η + H^{-1}A_W^T λ  (reuses HinvAwT, avoids an extra back-substitution)
  Vector x_vec = hinvEtaVec_ + HinvAwT * lambda;

  VectorValues x = toValues(x_vec);

  // Pack λ into VectorValues keyed by each constraint's dualKey().
  VectorValues duals;
  for (size_t r = 0; r < m; ++r) {
    Key dk = rowDualKey[r];
    if (!duals.exists(dk))
      duals.insert(dk, (Vector(1) << lambda[r]).finished());
  }

  return {x, duals};
}

// =============================================================================
// Active-set bookkeeping
// =============================================================================

std::tuple<double, int> QPSolver::computeStepSize(
    const InequalityFactorGraph& workingSet, const VectorValues& xk,
    const VectorValues& p, double maxAlpha) const {
  double minAlpha = maxAlpha;
  int closestFactorIx = -1;
  for (size_t i = 0; i < workingSet.size(); ++i) {
    const LinearInequality::shared_ptr& factor = workingSet.at(i);
    if (factor->active()) continue;
    double aTp = factor->dotProductRow(p);
    if (aTp <= 0) continue;
    double aTx = factor->dotProductRow(xk);
    double alpha = (factor->getb()[0] - aTx) / aTp;
    if (alpha < minAlpha) {
      closestFactorIx = static_cast<int>(i);
      minAlpha = alpha;
    }
  }
  return {minAlpha, closestFactorIx};
}

int QPSolver::identifyLeavingConstraint(
    const InequalityFactorGraph& workingSet,
    const VectorValues& lambdas) const {
  int worstFactorIx = -1;
  double maxLambda = 0.0;
  for (size_t i = 0; i < workingSet.size(); ++i) {
    const LinearInequality::shared_ptr& factor = workingSet.at(i);
    if (!factor->active()) continue;
    double lambda = lambdas.at(factor->dualKey())[0];
    if (lambda > maxLambda) {
      worstFactorIx = static_cast<int>(i);
      maxLambda = lambda;
    }
  }
  return worstFactorIx;
}

InequalityFactorGraph QPSolver::identifyActiveConstraints(
    const InequalityFactorGraph& inequalities,
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  InequalityFactorGraph workingSet;
  for (const LinearInequality::shared_ptr& factor : inequalities) {
    LinearInequality::shared_ptr wf(new LinearInequality(*factor));
    if (useWarmStart && duals.size() > 0) {
      if (duals.exists(wf->dualKey()))
        wf->activate();
      else
        wf->inactivate();
    } else {
      double error = wf->error(initialValues);
      if (error > 0) throw InfeasibleInitialValues();
      if (std::abs(error) < 1e-7)
        wf->activate();
      else
        wf->inactivate();
    }
    workingSet.push_back(wf);
  }
  return workingSet;
}

// =============================================================================
// Core iteration — Schur complement replaces GFG solve
// =============================================================================

QPSolver::State QPSolver::iterate(const State& state) const {
  // Solve KKT subproblem for current active set.
  // Unlike the GFG-based iterate(), solveKKT() returns the KKT multipliers
  // λ simultaneously, so no second solve is needed in the stall branch.
  auto [newValues, lambdas] = solveKKT(state.workingSet);

  if (newValues.equals(state.values, 1e-7)) {
    // Stalled: use the already-available KKT multipliers for the KKT check.
    int leavingFactor = identifyLeavingConstraint(state.workingSet, lambdas);
    if (leavingFactor < 0) {
      return State(newValues, lambdas, state.workingSet, true,
                   state.iterations + 1);
    } else {
      InequalityFactorGraph newWorkingSet = state.workingSet;
      newWorkingSet.at(leavingFactor)->inactivate();
      return State(newValues, lambdas, newWorkingSet, false,
                   state.iterations + 1);
    }
  } else {
    VectorValues p = newValues - state.values;
    const auto [alpha, factorIx] =
        computeStepSize(state.workingSet, state.values, p, 1.0 /*QP maxAlpha*/);
    InequalityFactorGraph newWorkingSet = state.workingSet;
    if (factorIx >= 0) newWorkingSet.at(factorIx)->activate();
    newValues = state.values + alpha * p;
    return State(newValues, state.duals, newWorkingSet, false,
                 state.iterations + 1);
  }
}

// =============================================================================
// Public optimize() interface
// =============================================================================

std::pair<VectorValues, VectorValues> QPSolver::optimize(
    const VectorValues& initialValues, const VectorValues& duals,
    bool useWarmStart) const {
  InequalityFactorGraph workingSet = identifyActiveConstraints(
      problem_.inequalities, initialValues, duals, useWarmStart);
  State state(initialValues, duals, workingSet, false, 0);
  while (!state.converged) state = iterate(state);
  return {state.values, state.duals};
}

std::pair<VectorValues, VectorValues> QPSolver::optimize() const {
  QPInitSolver initSolver(problem_);
  return optimize(initSolver.solve());
}

std::tuple<VectorValues, VectorValues, QPSolver::State>
QPSolver::optimizeWithState(const VectorValues& initialValues,
                                const VectorValues& duals,
                                bool useWarmStart) const {
  InequalityFactorGraph workingSet = identifyActiveConstraints(
      problem_.inequalities, initialValues, duals, useWarmStart);
  State state(initialValues, duals, workingSet, false, 0);
  while (!state.converged) state = iterate(state);
  return {state.values, state.duals, state};
}

}  // namespace gtsam
