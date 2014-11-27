/*
 * QPSolver.h
 * @brief: A quadratic programming solver implements the active set method
 * @date: Apr 15, 2014
 * @author: thduynguyen
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <vector>
#include <set>

namespace gtsam {

/**
 * This class implements the active set method to solve quadratic programming problems
 * encoded in a GaussianFactorGraph with special mixed constrained noise models, in which
 * a negative sigma denotes an inequality <=0 constraint,
 * a zero sigma denotes an equality =0 constraint,
 * and a positive sigma denotes a normal Gaussian noise model.
 */
class QPSolver {

  class Hessians: public FactorGraph<HessianFactor> {
  };

  const GaussianFactorGraph& graph_; //!< the original graph, can't be modified!
  FastVector<size_t> constraintIndices_; //!< Indices of constrained factors in the original graph
  Hessians freeHessians_; //!< unconstrained Hessians of constrained variables
  VariableIndex freeHessianFactorIndex_; //!< indices of unconstrained Hessian factors of constrained variables
                                         // gtsam calls it "VariableIndex", but I think FactorIndex
                                         // makes more sense, because it really stores factor indices.
  VariableIndex fullFactorIndices_; //!< indices of factors involving each variable.
                                    // gtsam calls it "VariableIndex", but I think FactorIndex
                                    // makes more sense, because it really stores factor indices.

public:
  /// Constructor
  QPSolver(const GaussianFactorGraph& graph);

  /// Return indices of all constrained factors
  FastVector<size_t> constraintIndices() const {
    return constraintIndices_;
  }

  /// Return the Hessian factor graph of constrained variables
  const Hessians& freeHessiansOfConstrainedVars() const {
    return freeHessians_;
  }

  /**
   * Build the dual graph to solve for the Lagrange multipliers.
   *
   * The Lagrangian function is:
   *        L(X,lambdas) = f(X) - \sum_k lambda_k * c_k(X),
   * where the unconstrained part is
   *        f(X) = 0.5*X'*G*X - X'*g + 0.5*f0
   * and the linear equality constraints are
   *        c1(X), c2(X), ..., cm(X)
   *
   * Take the derivative of L wrt X at the solution and set it to 0, we have
   *    \grad f(X) = \sum_k lambda_k * \grad c_k(X)   (*)
   *
   * For each set of rows of (*) corresponding to a variable xi involving in some constraints
   * we have:
   *    \grad f(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
   *    \grad c_k(xi) = \frac{\partial c_k}{\partial xi}'
   *
   * Note: If xi does not involve in any constraint, we have the trivial condition
   * \grad f(Xi) = 0, which should be satisfied as a usual condition for unconstrained variables.
   *
   * So each variable xi involving in some constraints becomes a linear factor A*lambdas - b = 0
   * on the constraints' lambda multipliers, as follows:
   *    - The jacobian term A_k for each lambda_k is \grad c_k(xi)
   *    - The constant term b is \grad f(xi), which can be computed from all unconstrained
   *    Hessian factors connecting to xi: \grad f(xi) = \sum_j G_ij*xj - gi
   */
  GaussianFactorGraph buildDualGraph(const GaussianFactorGraph& graph,
      const VectorValues& x0, bool useLeastSquare = false) const;

  /**
   * The goal of this function is to find currently active inequality constraints
   * that violate the condition to be active. The one that violates the condition
   * the most will be removed from the active set. See Nocedal06book, pg 469-471
   *
   * Find the BAD active inequality that pulls x strongest to the wrong direction
   * of its constraint (i.e. it is pulling towards >0, while its feasible region is <=0)
   *
   * For active inequality constraints (those that are enforced as equality constraints
   * in the current working set), we want lambda < 0.
   * This is because:
   *   - From the Lagrangian L = f - lambda*c, we know that the constraint force
   *     is (lambda * \grad c) = \grad f. Intuitively, to keep the solution x stay
   *     on the constraint surface, the constraint force has to balance out with
   *     other unconstrained forces that are pulling x towards the unconstrained
   *     minimum point. The other unconstrained forces are pulling x toward (-\grad f),
   *     hence the constraint force has to be exactly \grad f, so that the total
   *     force is 0.
   *   - We also know that  at the constraint surface c(x)=0, \grad c points towards + (>= 0),
   *     while we are solving for - (<=0) constraint.
   *   - We want the constraint force (lambda * \grad c) to pull x towards the - (<=0) direction
   *     i.e., the opposite direction of \grad c where the inequality constraint <=0 is satisfied.
   *     That means we want lambda < 0.
   *   - This is because when the constrained force pulls x towards the infeasible region (+),
   *     the unconstrained force is pulling x towards the opposite direction into
   *     the feasible region (again because the total force has to be 0 to make x stay still)
   *     So we can drop this constraint to have a lower error but feasible solution.
   *
   * In short, active inequality constraints with lambda > 0 are BAD, because they
   * violate the condition to be active.
   *
   * And we want to remove the worst one with the largest lambda from the active set.
   *
   */
  std::pair<int, int> findWorstViolatedActiveIneq(
      const VectorValues& lambdas) const;

  /**
   * Deactivate or activate an inequality constraint in place
   * Warning: modify in-place to avoid copy/clone
   * @return true if update successful
   */
  bool updateWorkingSetInplace(GaussianFactorGraph& workingGraph, int factorIx,
      int sigmaIx, double newSigma) const;

  /**
   * Compute step size alpha for the new solution x' = xk + alpha*p, where alpha \in [0,1]
   *
   *    @return a tuple of (alpha, factorIndex, sigmaIndex) where (factorIndex, sigmaIndex)
   *            is the constraint that has minimum alpha, or (-1,-1) if alpha = 1.
   *            This constraint will be added to the working set and become active
   *            in the next iteration
   */
  boost::tuple<double, int, int> computeStepSize(
      const GaussianFactorGraph& workingGraph, const VectorValues& xk,
      const VectorValues& p) const;

  /** Iterate 1 step, modify workingGraph and currentSolution *IN PLACE* !!! */
  bool iterateInPlace(GaussianFactorGraph& workingGraph,
      VectorValues& currentSolution, VectorValues& lambdas) const;

  /** Optimize with a provided initial values
   * For this version, it is the responsibility of the caller to provide
   * a feasible initial value, otherwise the solution will be wrong.
   * If you don't know a feasible initial value, use the other version
   * of optimize().
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initials) const;

  /** Optimize without an initial value.
   * This version of optimize will try to find a feasible initial value by solving
   * an LP program before solving this QP graph.
   * TODO: If no feasible initial point exists, it should throw an InfeasibilityException!
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize() const;

  /**
   * Create initial values for the LP subproblem
   * @return initial values and the key for the first slack variable
   */
  std::pair<VectorValues, Key> initialValuesLP() const;

  /// Create coefficients for the LP subproblem's objective function as the sum of slack var
  VectorValues objectiveCoeffsLP(Key firstSlackKey) const;

  /// Build constraints and slacks' lower bounds for the LP subproblem
  std::pair<GaussianFactorGraph::shared_ptr, VectorValues> constraintsLP(
      Key firstSlackKey) const;

  /// Find a feasible initial point
  std::pair<bool, VectorValues> findFeasibleInitialValues() const;

private:

  /// Collect all free Hessians involving constrained variables into a graph
  void findUnconstrainedHessiansOfConstrainedVars(
      const std::set<Key>& constrainedVars);

};

} /* namespace gtsam */
