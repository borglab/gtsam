/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConstrainedSolverAdapter.h
 * @brief   Internal adapters from legacy unstable LP/QP models to constrained.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/LpProblem.h>
#include <gtsam/constrained/QpProblem.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/linear/LP.h>
#include <gtsam_unstable/linear/QP.h>

#include <map>
#include <stdexcept>

namespace gtsam {
namespace unstable_solver_adapter {

using KeyDimMap = std::map<Key, size_t>;

/* ************************************************************************* */
inline void AddOrCheckDim(KeyDimMap* keyDims, Key key, size_t dim) {
  const auto [it, inserted] = keyDims->emplace(key, dim);
  if (!inserted && it->second != dim) {
    throw std::invalid_argument(
        "ConstrainedSolverAdapter: inconsistent key dimensions.");
  }
}

/* ************************************************************************* */
inline void AddFactorDims(KeyDimMap* keyDims, const GaussianFactor& factor) {
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    AddOrCheckDim(keyDims, *it, static_cast<size_t>(factor.getDim(it)));
  }
}

/* ************************************************************************* */
template <typename GRAPH>
inline void AddGraphDims(KeyDimMap* keyDims, const GRAPH& graph) {
  for (const auto& factor : graph) {
    if (factor) {
      AddFactorDims(keyDims, *factor);
    }
  }
}

/* ************************************************************************* */
inline KeyDimMap CollectKeyDims(const QP& qp) {
  KeyDimMap keyDims;
  AddGraphDims(&keyDims, qp.cost);
  AddGraphDims(&keyDims, qp.equalities);
  AddGraphDims(&keyDims, qp.inequalities);
  return keyDims;
}

/* ************************************************************************* */
inline KeyDimMap CollectKeyDims(const LP& lp) {
  KeyDimMap keyDims;
  AddFactorDims(&keyDims, lp.cost);
  AddGraphDims(&keyDims, lp.equalities);
  AddGraphDims(&keyDims, lp.inequalities);
  return keyDims;
}

/* ************************************************************************* */
inline Values ToValues(const VectorValues& vectorValues) {
  Values values;
  for (const auto& [key, vector] : vectorValues) {
    values.insert(key, vector);
  }
  return values;
}

/* ************************************************************************* */
inline VectorValues ToVectorValues(const Values& values,
                                   const KeyDimMap& keyDims) {
  VectorValues vectorValues;
  for (const auto& [key, _] : keyDims) {
    if (values.exists(key)) {
      vectorValues.insert(key, values.at<Vector>(key));
    }
  }
  return vectorValues;
}

/* ************************************************************************* */
inline QpProblem ToConstrainedProblem(const QP& qp) {
  QpProblem problem;
  for (const auto& factor : qp.cost) {
    if (factor) {
      problem.addCost(*factor);
    }
  }
  for (const auto& factor : qp.equalities) {
    if (factor) {
      problem.addConstraint(
          LinearConstraint::Equal(static_cast<const JacobianFactor&>(*factor)));
    }
  }
  for (const auto& factor : qp.inequalities) {
    if (factor) {
      problem.addConstraint(LinearConstraint::LessEqual(
          static_cast<const JacobianFactor&>(*factor)));
    }
  }
  return problem;
}

/* ************************************************************************* */
inline LpProblem ToConstrainedProblem(const LP& lp) {
  LpProblem problem;
  problem.addCost(static_cast<const JacobianFactor&>(lp.cost));
  for (const auto& factor : lp.equalities) {
    if (factor) {
      problem.addConstraint(
          LinearConstraint::Equal(static_cast<const JacobianFactor&>(*factor)));
    }
  }
  for (const auto& factor : lp.inequalities) {
    if (factor) {
      problem.addConstraint(LinearConstraint::LessEqual(
          static_cast<const JacobianFactor&>(*factor)));
    }
  }
  return problem;
}

/* ************************************************************************* */
inline ActiveSetSolver::State WarmStartState(
    const InequalityFactorGraph& inequalities, const VectorValues& duals) {
  ActiveSetSolver::State state;
  state.activeInequalityRows.reserve(inequalities.size());
  for (const auto& factor : inequalities) {
    state.activeInequalityRows.push_back(factor &&
                                         duals.exists(factor->dualKey()));
  }
  return state;
}

/* ************************************************************************* */
inline void AddEqualityDuals(VectorValues* duals,
                             const EqualityFactorGraph& equalities,
                             const std::vector<Vector>& multipliers) {
  for (size_t index = 0;
       index < equalities.size() && index < multipliers.size(); ++index) {
    const auto& factor = equalities.at(index);
    if (factor) {
      duals->insert_or_assign(factor->dualKey(), multipliers[index]);
    }
  }
}

/* ************************************************************************* */
inline void AddInequalityDuals(VectorValues* duals,
                               const InequalityFactorGraph& inequalities,
                               const std::vector<Vector>& multipliers,
                               const std::vector<bool>& activeRows) {
  for (size_t index = 0;
       index < inequalities.size() && index < multipliers.size() &&
       index < activeRows.size();
       ++index) {
    const auto& factor = inequalities.at(index);
    if (factor && activeRows[index]) {
      duals->insert_or_assign(factor->dualKey(), multipliers[index]);
    }
  }
}

/* ************************************************************************* */
inline VectorValues ToLegacyDuals(const QP& qp,
                                  const ActiveSetSolver::State& state) {
  VectorValues duals;
  AddEqualityDuals(&duals, qp.equalities, state.equalityMultipliers);
  AddInequalityDuals(&duals, qp.inequalities, state.inequalityMultipliers,
                     state.activeInequalityRows);
  return duals;
}

/* ************************************************************************* */
inline VectorValues ToLegacyDuals(const LP& lp,
                                  const ActiveSetSolver::State& state) {
  VectorValues duals;
  AddEqualityDuals(&duals, lp.equalities, state.equalityMultipliers);
  AddInequalityDuals(&duals, lp.inequalities, state.inequalityMultipliers,
                     state.activeInequalityRows);
  return duals;
}

}  // namespace unstable_solver_adapter
}  // namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
