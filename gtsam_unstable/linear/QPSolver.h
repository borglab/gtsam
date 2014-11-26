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
  const GaussianFactorGraph& graph_; //!< the original graph, can't be modified!
  FastVector<size_t> constraintIndices_; //!< Indices of constrained factors in the original graph
  GaussianFactorGraph::shared_ptr freeHessians_; //!< unconstrained Hessians of constrained variables
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
  GaussianFactorGraph::shared_ptr freeHessiansOfConstrainedVars() const {
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
   * Find the BAD active ineq that pulls x strongest to the wrong direction of its constraint
   * (i.e. it is pulling towards >0, while its feasible region is <=0)
   *
   * For active ineq constraints (those that are enforced as eq constraints now
   * in the working set), we want lambda < 0.
   * This is because:
   *     - From the Lagrangian L = f - lambda*c, we know that the constraint force is
   *     (lambda * \grad c) = \grad f, because it cancels out the unconstrained
   *     unconstrained force (-\grad f), which is pulling x in the opposite direction
   *     of \grad f towards the unconstrained minimum point
   *     - We also know that  at the constraint surface \grad c points toward + (>= 0),
   *     while we are solving for - (<=0) constraint
   *     - So, we want the constraint force (lambda * \grad c) to to pull x
   *     towards the opposite direction of \grad c, i.e. towards the area
   *     where the ineq constraint <=0 is satisfied.
   *     - Hence, we want lambda < 0
   *
   * So active ineqs with lambda > 0 are BAD. And we want the worst one with the largest lambda.
   *
   */
  std::pair<int, int> findWorstViolatedActiveIneq(
      const VectorValues& lambdas) const;

  /**
   * Deactivate or activate an ineq constraint in place
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

  /// Convert a Gaussian factor to a jacobian. return empty shared ptr if failed
  /// TODO: Move to GaussianFactor?
  static JacobianFactor::shared_ptr toJacobian(
      const GaussianFactor::shared_ptr& factor) {
    JacobianFactor::shared_ptr jacobian(
        boost::dynamic_pointer_cast<JacobianFactor>(factor));
    return jacobian;
  }

  /// Convert a Gaussian factor to a Hessian. Return empty shared ptr if failed
  /// TODO: Move to GaussianFactor?
  static HessianFactor::shared_ptr toHessian(
      const GaussianFactor::shared_ptr factor) {
    HessianFactor::shared_ptr hessian(
        boost::dynamic_pointer_cast<HessianFactor>(factor));
    return hessian;
  }

private:
  /// Collect all free Hessians involving constrained variables into a graph
  GaussianFactorGraph::shared_ptr unconstrainedHessiansOfConstrainedVars(
      const GaussianFactorGraph& graph,
      const std::set<Key>& constrainedVars) const;

};

} /* namespace gtsam */
