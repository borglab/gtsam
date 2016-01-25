#pragma once

namespace gtsam {
/**
 * Abstract class to solve for an initial value of an LP problem
 */
class LPInitSolver {
protected:
  const LP& lp_;
  const LPSolver& lpSolver_;

public:
  LPInitSolver(const LPSolver& lpSolver) :
      lp_(lpSolver.lp()), lpSolver_(lpSolver) {
  }
  virtual ~LPInitSolver() {
  }
  ;
  virtual VectorValues solve() const = 0;
};
}
