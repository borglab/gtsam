/**
 * @file     QPInitSolver.h
 * @brief    This QPInitSolver implements the strategy in Matlab.
 * @author   Duy Nguyen Ta
 * @author   Ivan Dario Jimenez
 * @date     6/16/16
 */

#pragma once

#include <gtsam_unstable/linear/LPInitSolver.h>

namespace gtsam {

/**
 * This class finds a feasible solution for a QP problem.
 * This uses the Matlab strategy for initialization
 * For details, see
 * http://www.mathworks.com/help/optim/ug/quadratic-programming-algorithms.html#brrzwpf-22
 */
class QPInitSolver {
    const QP& qp_;
public:
    /// Constructor with a QP problem
    QPInitSolver(const QP& qp) : qp_(qp) {}

    /**
     * @return a feasible initialization point
     */
    VectorValues solve() const {
      // Make an LP with any linear cost function. It doesn't matter for
      // initialization.
      LP initProblem;
      // make an unrelated key for a random variable cost
      Key newKey = maxKey(qp_) + 1;
      initProblem.cost = LinearCost(newKey, Vector::Ones(1));
      initProblem.equalities = qp_.equalities;
      initProblem.inequalities = qp_.inequalities;
      LPInitSolver initSolver(initProblem);
      return initSolver.solve();
    }
};


}