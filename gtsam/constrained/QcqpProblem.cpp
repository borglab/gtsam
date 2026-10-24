/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QcqpProblem.cpp
 * @brief   QCQP problem implementations.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace gtsam {
namespace {

using QuadraticConstraintIndex =
    std::unordered_map<Key, std::vector<QuadraticConstraint>>;

/* ************************************************************************* */
bool SameQuadraticEquality(const QuadraticConstraint& first,
                           const QuadraticConstraint& second) {
  return first.key() == second.key() && first.A().isApprox(second.A(), 0.0) &&
         first.a().isApprox(second.a(), 0.0) && first.b() == second.b() &&
         first.sigma() == second.sigma();
}

// Tolerance for checking a fixed variable against its constraints.
constexpr double kFixedVariableTolerance = 1e-9;

/* ************************************************************************* */
bool InvolvesFixedVariable(const KeyVector& keys, const Values& fixedVariables) {
  return std::any_of(keys.begin(), keys.end(), [&fixedVariables](Key key) {
    return fixedVariables.exists(key);
  });
}

/* ************************************************************************* */
// A FrobeniusPrior with a constrained noise model fixes its variable. Record
// the variable's QCQP value v after checking it against the constraints the
// variable would carry, so that a target M that is not a valid rotation or
// pose is rejected rather than silently substituted.
template <typename T>
bool RecordFixedVariable(const NonlinearFactor& factor,
                         Values* fixedVariables) {
  const auto* frobeniusPrior = dynamic_cast<const FrobeniusPrior<T>*>(&factor);
  if (!frobeniusPrior || !frobeniusPrior->noiseModel() ||
      !frobeniusPrior->noiseModel()->isConstrained()) {
    return false;
  }

  // Lower the FrobeniusPrior on its own, only to read its lifted target v
  // from the emitted constraint x = v.
  NonlinearFactorGraph frobeniusPriorCosts;
  NonlinearEqualityConstraints frobeniusPriorConstraints;
  frobeniusPrior->qcqpFactors(&frobeniusPriorCosts, &frobeniusPriorConstraints,
                              1);
  Matrix v;
  for (const auto& constraint : frobeniusPriorConstraints) {
    if (const auto* linear =
            dynamic_cast<const LinearEqualityConstraintFactor*>(
                constraint.get())) {
      v = linear->linearConstraint().factor().getb();
    }
  }

  const Key key = frobeniusPrior->key();
  Values fixedVariable;
  fixedVariable.insert(key, v);
  if (frobeniusPriorConstraints.violationNorm(fixedVariable) >
      kFixedVariableTolerance) {
    throw std::invalid_argument(
        "QcqpProblem: a hard FrobeniusPrior must target a valid value of its "
        "variable type.");
  }

  if (fixedVariables->exists(key)) {
    if ((fixedVariables->at<Matrix>(key) - v).norm() >
        kFixedVariableTolerance) {
      throw std::invalid_argument(
          "QcqpProblem: conflicting hard FrobeniusPriors on one variable.");
    }
    return true;
  }
  fixedVariables->insert(key, v);
  return true;
}

/* ************************************************************************* */
// Substitute the fixed variables into the QCQP costs. A cost
// ½xᵀGx − xᵀg + ½f, split into free positions F and fixed positions K with
// x_K = v, becomes a cost on x_F with
//   G' = G_FF,  g' = g_F − G_FK v,  f' = f + vᵀG_KK v − 2g_Kᵀv.
// A binary cost with a fixed variable thus becomes a unary cost on the other
// variable, which keeps the gauge fixed. A cost on fixed variables only
// leaves f', which is added to *constant.
NonlinearFactorGraph SubstituteFixedVariables(const NonlinearFactorGraph& costs,
                                              const Values& fixedVariables,
                                              double* constant) {
  NonlinearFactorGraph substitutedCosts;
  for (const auto& factor : costs) {
    if (!InvolvesFixedVariable(factor->keys(), fixedVariables)) {
      substitutedCosts.push_back(factor);
      continue;
    }
    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::invalid_argument(
          "QcqpProblem: fixed variables can only be substituted into QpCost "
          "costs.");
    }

    const HessianFactor& hessian = cost->hessianFactor();
    KeyVector freeKeys;
    std::vector<DenseIndex> freeDims;
    std::vector<Eigen::Index> F, K;
    Vector v_K(0);
    DenseIndex offset = 0;
    for (auto it = hessian.begin(); it != hessian.end(); ++it) {
      const DenseIndex dim = hessian.getDim(it);
      auto& positions = fixedVariables.exists(*it) ? K : F;
      for (DenseIndex i = 0; i < dim; ++i) positions.push_back(offset + i);
      if (fixedVariables.exists(*it)) {
        const Matrix& value = fixedVariables.at<Matrix>(*it);
        if (value.size() != dim) {
          throw std::invalid_argument(
              "QcqpProblem: fixed variable dimension does not match a cost.");
        }
        v_K.conservativeResize(v_K.size() + dim);
        v_K.tail(dim) = Eigen::Map<const Vector>(value.data(), dim);
      } else {
        freeKeys.push_back(*it);
        freeDims.push_back(dim);
      }
      offset += dim;
    }

    const Matrix G = hessian.information();
    const Vector g = hessian.linearTerm();
    const double f = hessian.constantTerm();
    const Matrix G_FK = G(F, K);
    const double fSubstituted =
        f + v_K.dot(G(K, K) * v_K) - 2.0 * g(K).dot(v_K);
    if (freeKeys.empty()) {
      *constant += fSubstituted;
      continue;
    }

    // Augmented [G' g'; g'ᵀ f'] with one block per free variable plus a
    // size-1 block, as in the factors' own QCQP costs.
    const DenseIndex n = F.size();
    Matrix Q = Matrix::Zero(n + 1, n + 1);
    Q.topLeftCorner(n, n) = G(F, F);
    Q.topRightCorner(n, 1) = g(F) - G_FK * v_K;
    Q.bottomLeftCorner(1, n) = Q.topRightCorner(n, 1).transpose();
    Q(n, n) = fSubstituted;
    freeDims.push_back(1);
    const SymmetricBlockMatrix blockQ(freeDims, Q);
    substitutedCosts.emplace_shared<QpCost>(HessianFactor(freeKeys, blockQ));
  }
  return substitutedCosts;
}

/* ************************************************************************* */
// Merge one factor's equality constraints, dropping duplicate quadratic
// equalities. Constraints on a fixed variable are dropped after checking that
// its value satisfies them.
void MergeEqualityConstraints(
    const NonlinearEqualityConstraints& source, const Values& fixedVariables,
    NonlinearEqualityConstraints* destination,
    QuadraticConstraintIndex* quadraticConstraintIndex) {
  for (const auto& factor : source) {
    if (InvolvesFixedVariable(factor->keys(), fixedVariables)) {
      if (factor->violation(fixedVariables) > kFixedVariableTolerance) {
        throw std::invalid_argument(
            "QcqpProblem: a fixed variable violates one of its constraints.");
      }
      continue;
    }
    const auto quadratic =
        std::dynamic_pointer_cast<QuadraticEqualityConstraintFactor>(factor);
    if (!quadratic) {
      destination->push_back(factor);
      continue;
    }

    const QuadraticConstraint& constraint = quadratic->quadraticConstraint();
    auto& sameKeyConstraints = (*quadraticConstraintIndex)[constraint.key()];
    const bool alreadyPresent =
        std::any_of(sameKeyConstraints.begin(), sameKeyConstraints.end(),
                    [&constraint](const QuadraticConstraint& existing) {
                      return SameQuadraticEquality(existing, constraint);
                    });
    if (!alreadyPresent) {
      destination->push_back(factor);
      sameKeyConstraints.push_back(constraint);
    }
  }
}

}  // namespace

/* ************************************************************************* */
QcqpProblem::QcqpProblem(const NonlinearFactorGraph& graph,
                         size_t columnDimension) {
  if (columnDimension == 0) {
    throw std::invalid_argument(
        "QcqpProblem: columnDimension must be positive.");
  }

  // Pass 1, D=1 only: each hard FrobeniusPrior fixes its variable. A
  // separate pass is needed because a cost on a fixed variable can precede
  // its FrobeniusPrior in the graph.
  std::vector<bool> isHardFrobeniusPrior(graph.size(), false);
  if (columnDimension == 1) {
    for (size_t i = 0; i < graph.size(); ++i) {
      if (!graph[i]) continue;
      isHardFrobeniusPrior[i] =
          RecordFixedVariable<Rot2>(*graph[i], &fixedVariables_) ||
          RecordFixedVariable<Rot3>(*graph[i], &fixedVariables_) ||
          RecordFixedVariable<Pose2>(*graph[i], &fixedVariables_) ||
          RecordFixedVariable<Pose3>(*graph[i], &fixedVariables_);
    }
  }

  // Pass 2: lower every other factor, substituting the fixed variables.
  QuadraticConstraintIndex quadraticConstraintIndex;
  double constant = 0.0;
  for (size_t i = 0; i < graph.size(); ++i) {
    if (!graph[i] || isHardFrobeniusPrior[i]) continue;

    NonlinearFactorGraph factorCosts;
    NonlinearEqualityConstraints factorConstraints;
    graph[i]->qcqpFactors(&factorCosts, &factorConstraints, columnDimension);
    costs_.add(
        SubstituteFixedVariables(factorCosts, fixedVariables_, &constant));
    MergeEqualityConstraints(factorConstraints, fixedVariables_,
                             &eqConstraints_, &quadraticConstraintIndex);
  }

  // Keep the objective exact: fold the constant left by costs on fixed
  // variables only into the first remaining cost.
  if (constant != 0.0) {
    for (size_t i = 0; i < costs_.size(); ++i) {
      const auto* cost = dynamic_cast<const QpCost*>(costs_[i].get());
      if (!cost) continue;
      HessianFactor hessian = cost->hessianFactor();
      hessian.constantTerm() += constant;
      costs_.replace(i, std::make_shared<QpCost>(hessian));
      break;
    }
  }
}

/* ************************************************************************* */
void QcqpProblem::addCost(const QpCost& cost) {
  if (InvolvesFixedVariable(cost.keys(), fixedVariables_)) {
    throw std::invalid_argument(
        "QcqpProblem::addCost: the cost involves a fixed variable.");
  }
  costs_.emplace_shared<QpCost>(cost);
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const LinearConstraint& constraint) {
  if (InvolvesFixedVariable(constraint.factor().keys(), fixedVariables_)) {
    throw std::invalid_argument(
        "QcqpProblem::addConstraint: the constraint involves a fixed "
        "variable.");
  }
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const QuadraticConstraint& constraint) {
  if (fixedVariables_.exists(constraint.key())) {
    throw std::invalid_argument(
        "QcqpProblem::addConstraint: the constraint involves a fixed "
        "variable.");
  }
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

}  // namespace gtsam
