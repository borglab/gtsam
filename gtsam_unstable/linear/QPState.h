//
// Created by ivan on 1/25/16.
//

#pragma once

namespace gtsam {
/// This struct holds the state of QPSolver at each iteration
struct QPState {
  VectorValues values;
  VectorValues duals;
  InequalityFactorGraph workingSet;
  bool converged;
  size_t iterations;

  /// default constructor
  QPState() :
      values(), duals(), workingSet(), converged(false), iterations(0) {
  }

  /// constructor with initial values
  QPState(const VectorValues& initialValues, const VectorValues& initialDuals,
      const InequalityFactorGraph& initialWorkingSet, bool _converged,
      size_t _iterations) :
      values(initialValues), duals(initialDuals), workingSet(initialWorkingSet), converged(
          _converged), iterations(_iterations) {
  }
};
}
