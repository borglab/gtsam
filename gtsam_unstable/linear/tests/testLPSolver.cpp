/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testQPSolver.cpp
 * @brief Test simple QP solver for a linear inequality constraint
 * @date Apr 10, 2014
 * @author Duy-Nguyen Ta
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam_unstable/linear/LinearCost.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

namespace gtsam {

/* ************************************************************************* */
/** An exception indicating that the provided initial value is infeasible */
class InfeasibleInitialValues: public ThreadsafeException<
    InfeasibleInitialValues> {
public:
  InfeasibleInitialValues() {
  }
  virtual ~InfeasibleInitialValues() throw () {
  }

  virtual const char* what() const throw () {
    if (description_.empty()) description_ =
        "An infeasible initial value was provided for the solver.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};

/// Throw when the problem is either infeasible or unbounded
class InfeasibleOrUnboundedProblem: public ThreadsafeException<
InfeasibleOrUnboundedProblem> {
public:
  InfeasibleOrUnboundedProblem() {
  }
  virtual ~InfeasibleOrUnboundedProblem() throw () {
  }

  virtual const char* what() const throw () {
    if (description_.empty()) description_ =
        "The problem is either infeasible or unbounded.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};


struct LP {
  LinearCost cost; //!< Linear cost factor
  EqualityFactorGraph equalities; //!< Linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< Linear inequality constraints: cI(x) <= 0

  /// check feasibility
  bool isFeasible(const VectorValues& x) const {
    return (equalities.error(x) == 0 && inequalities.error(x) == 0);
  }

  /// print
  void print(const string& s = "") const {
    std::cout << s << std::endl;
    cost.print("Linear cost: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }

  /// equals
  bool equals(const LP& other, double tol = 1e-9) const {
    return cost.equals(other.cost)
        && equalities.equals(other.equalities)
        && inequalities.equals(other.inequalities);
  }

  typedef boost::shared_ptr<LP> shared_ptr;
};

/// traits
template<> struct traits<LP> : public Testable<LP> {};

/// This struct holds the state of QPSolver at each iteration
struct LPState {
  VectorValues values;
  VectorValues duals;
  InequalityFactorGraph workingSet;
  bool converged;
  size_t iterations;

  /// default constructor
  LPState() :
      values(), duals(), workingSet(), converged(false), iterations(0) {
  }

  /// constructor with initial values
  LPState(const VectorValues& initialValues, const VectorValues& initialDuals,
      const InequalityFactorGraph& initialWorkingSet, bool _converged,
      size_t _iterations) :
      values(initialValues), duals(initialDuals), workingSet(initialWorkingSet), converged(
          _converged), iterations(_iterations) {
  }
};

typedef std::map<Key, size_t> KeyDimMap;
typedef std::vector<std::pair<Key, Matrix> > TermsContainer;

class LPSolver {
  const LP& lp_; //!< the linear programming problem
  GaussianFactorGraph baseGraph_; //!< unchanged factors needed in every iteration
  VariableIndex costVariableIndex_, equalityVariableIndex_,
      inequalityVariableIndex_; //!< index to corresponding factors to build dual graphs
  FastSet<Key> constrainedKeys_; //!< all constrained keys, will become factors in dual graphs
  KeyDimMap keysDim_; //!< key-dim map of all variables in the constraints, used to create zero priors

public:
  LPSolver(const LP& lp) :
      lp_(lp) {
    // Push back factors that are the same in every iteration to the base graph.
    // Those include the equality constraints and zero priors for keys that are not
    // in the cost
    baseGraph_.push_back(lp_.equalities);

    // Collect key-dim map of all variables in the constraints to create their zero priors later
    keysDim_ = collectKeysDim(lp_.equalities);
    KeyDimMap keysDim2 = collectKeysDim(lp_.inequalities);
    keysDim_.insert(keysDim2.begin(), keysDim2.end());

    // Create and push zero priors of constrained variables that do not exist in the cost function
    baseGraph_.push_back(*createZeroPriors(lp_.cost.keys(), keysDim_));

    // Variable index
    equalityVariableIndex_ = VariableIndex(lp_.equalities);
    inequalityVariableIndex_ = VariableIndex(lp_.inequalities);
    constrainedKeys_ = lp_.equalities.keys();
    constrainedKeys_.merge(lp_.inequalities.keys());
  }

  const LP& lp() const { return lp_; }
  const KeyDimMap& keysDim() const { return keysDim_; }

  //******************************************************************************
  template<class LinearGraph>
  KeyDimMap collectKeysDim(const LinearGraph& linearGraph) const {
    KeyDimMap keysDim;
    BOOST_FOREACH(const typename LinearGraph::sharedFactor& factor, linearGraph) {
      if (!factor) continue;
      BOOST_FOREACH(Key key, factor->keys())
        keysDim[key] = factor->getDim(factor->find(key));
    }
    return keysDim;
  }

  //******************************************************************************
  /**
   * Create a zero prior for any keys in the graph that don't exist in the cost
   */
  GaussianFactorGraph::shared_ptr createZeroPriors(const KeyVector& costKeys,
      const KeyDimMap& keysDim ) const {
    GaussianFactorGraph::shared_ptr graph(new GaussianFactorGraph());
    BOOST_FOREACH(Key key, keysDim | boost::adaptors::map_keys) {
      if (find(costKeys.begin(), costKeys.end(), key) == costKeys.end()) {
        size_t dim = keysDim.at(key);
        graph->push_back(JacobianFactor(key, eye(dim), zero(dim)));
      }
    }
    return graph;
  }

  //******************************************************************************
  LPState iterate(const LPState& state) const {
    static bool debug = false;

    // Solve with the current working set
    // LP: project the objective neggradient to the constraint's null space
    // to find the direction to move
    VectorValues newValues = solveWithCurrentWorkingSet(state.values,
        state.workingSet);
//    if (debug) state.workingSet.print("Working set:");
    if (debug) (newValues - state.values).print("New direction:");

    // If we CAN'T move further
    // LP: projection on the constraints' nullspace is zero: we are at a vertex
    if (newValues.equals(state.values, 1e-7)) {
      // Find and remove the bad ineq constraint by computing its lambda
      // Compute lambda from the dual graph
      // LP: project the objective's gradient onto each constraint gradient to obtain the dual scaling factors
      //	is it true??
      if (debug) cout << "Building dual graph..." << endl;
      GaussianFactorGraph::shared_ptr dualGraph = buildDualGraph(
          state.workingSet, newValues);
      if (debug) dualGraph->print("Dual graph: ");
      VectorValues duals = dualGraph->optimize();
      if (debug) duals.print("Duals :");

      // LP: see which ineq constraint has wrong pulling direction, i.e., dual < 0
      int leavingFactor = identifyLeavingConstraint(state.workingSet, duals);
      if (debug) cout << "leavingFactor: " << leavingFactor << endl;

      // If all inequality constraints are satisfied: We have the solution!!
      if (leavingFactor < 0) {
        // TODO If we still have infeasible equality constraints: the problem is over-constrained. No solution!
        // ...
        return LPState(newValues, duals, state.workingSet, true,
            state.iterations + 1);
      }
      else {
        // Inactivate the leaving constraint
        // LP: remove the bad ineq constraint out of the working set
        InequalityFactorGraph newWorkingSet = state.workingSet;
        newWorkingSet.at(leavingFactor)->inactivate();
        return LPState(newValues, duals, newWorkingSet, false,
            state.iterations + 1);
      }
    }
    else {
      // If we CAN make some progress, i.e. p_k != 0
      // Adapt stepsize if some inactive constraints complain about this move
      // LP: projection on nullspace is NOT zero:
      // 		find and put a blocking inactive constraint to the working set,
      // 		otherwise the problem is unbounded!!!
      double alpha;
      int factorIx;
      VectorValues p = newValues - state.values;
      boost::tie(alpha, factorIx) = // using 16.41
          computeStepSize(state.workingSet, state.values, p);
      if (debug) cout << "alpha, factorIx: " << alpha << " " << factorIx << " "
          << endl;

      // also add to the working set the one that complains the most
      InequalityFactorGraph newWorkingSet = state.workingSet;
      if (factorIx >= 0) newWorkingSet.at(factorIx)->activate();

      // step!
      newValues = state.values + alpha * p;
      if (debug) newValues.print("New solution:");

      return LPState(newValues, state.duals, newWorkingSet, false,
          state.iterations + 1);
    }
  }

  //******************************************************************************
  /**
   * Create the factor ||x-xk - (-g)||^2 where xk is the current feasible solution
   * on the constraint surface and g is the gradient of the linear cost,
   * i.e. -g is the direction we wish to follow to decrease the cost.
   *
   * Essentially, we try to match the direction d = x-xk with -g as much as possible
   * subject to the condition that x needs to be on the constraint surface, i.e., d is
   * along the surface's subspace.
   *
   * The least-square solution of this quadratic subject to a set of linear constraints
   * is the projection of the gradient onto the constraints' subspace
   */
  GaussianFactorGraph::shared_ptr createLeastSquareFactors(
      const LinearCost& cost, const VectorValues& xk) const {
    GaussianFactorGraph::shared_ptr graph(new GaussianFactorGraph());
    KeyVector keys = cost.keys();

    for (LinearCost::const_iterator it = cost.begin(); it != cost.end(); ++it) {
      size_t dim = cost.getDim(it);
      Vector b = xk.at(*it) - cost.getA(it).transpose(); // b = xk-g
      graph->push_back(JacobianFactor(*it, eye(dim), b));
    }

    return graph;
  }

  //******************************************************************************
  VectorValues solveWithCurrentWorkingSet(const VectorValues& xk,
      const InequalityFactorGraph& workingSet) const {
    GaussianFactorGraph workingGraph = baseGraph_; // || X - Xk + g ||^2
    workingGraph.push_back(*createLeastSquareFactors(lp_.cost, xk));

    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, workingSet) {
      if (factor->active()) workingGraph.push_back(factor);
    }
    return workingGraph.optimize();
  }

  //******************************************************************************
  /// Collect the Jacobian terms for a dual factor
  template<typename FACTOR>
  TermsContainer collectDualJacobians(Key key,
      const FactorGraph<FACTOR>& graph,
      const VariableIndex& variableIndex) const {
    TermsContainer Aterms;
    if (variableIndex.find(key) != variableIndex.end()) {
      BOOST_FOREACH(size_t factorIx, variableIndex[key]) {
        typename FACTOR::shared_ptr factor = graph.at(factorIx);
        if (!factor->active()) continue;
        Matrix Ai = factor->getA(factor->find(key)).transpose();
        Aterms.push_back(std::make_pair(factor->dualKey(), Ai));
      }
    }
    return Aterms;
  }

  //******************************************************************************
  JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph& workingSet,
      const VectorValues& delta) const {

    // Transpose the A matrix of constrained factors to have the jacobian of the dual key
    TermsContainer Aterms = collectDualJacobians<LinearEquality>(key,
        lp_.equalities, equalityVariableIndex_);
    TermsContainer AtermsInequalities = collectDualJacobians<LinearInequality>(
        key, workingSet, inequalityVariableIndex_);
    Aterms.insert(Aterms.end(), AtermsInequalities.begin(),
        AtermsInequalities.end());

    // Collect the gradients of unconstrained cost factors to the b vector
    if (Aterms.size() > 0) {
      Vector b = zero(delta.at(key).size());
      Factor::const_iterator it = lp_.cost.find(key);
      if (it != lp_.cost.end()) b = lp_.cost.getA(it);
      return boost::make_shared<JacobianFactor>(Aterms, b); // compute the least-square approximation of dual variables
    }
    else {
      return boost::make_shared<JacobianFactor>();
    }
  }

  //******************************************************************************
  GaussianFactorGraph::shared_ptr buildDualGraph(
      const InequalityFactorGraph& workingSet,
      const VectorValues& delta) const {
    GaussianFactorGraph::shared_ptr dualGraph(new GaussianFactorGraph());
    BOOST_FOREACH(Key key, constrainedKeys_) {
      // Each constrained key becomes a factor in the dual graph
      JacobianFactor::shared_ptr dualFactor = createDualFactor(key, workingSet,
          delta);
      if (!dualFactor->empty()) dualGraph->push_back(dualFactor);
    }
    return dualGraph;
  }

  //******************************************************************************
  int identifyLeavingConstraint(const InequalityFactorGraph& workingSet,
      const VectorValues& duals) const {
    int worstFactorIx = -1;
    // preset the maxLambda to 0.0: if lambda is <= 0.0, the constraint is either
    // inactive or a good inequality constraint, so we don't care!
    double max_s = 0.0;
    for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
      const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
      if (factor->active()) {
        double s = duals.at(factor->dualKey())[0];
        if (s > max_s) {
          worstFactorIx = factorIx;
          max_s = s;
        }
      }
    }
    return worstFactorIx;
  }

  //******************************************************************************
  std::pair<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p) const {
    static bool debug = false;

    double minAlpha = std::numeric_limits<double>::infinity();
    int closestFactorIx = -1;
    for (size_t factorIx = 0; factorIx < workingSet.size(); ++factorIx) {
      const LinearInequality::shared_ptr& factor = workingSet.at(factorIx);
      double b = factor->getb()[0];
      // only check inactive factors
      if (!factor->active()) {
        // Compute a'*p
        double aTp = factor->dotProductRow(p);

        // Check if  a'*p >0. Don't care if it's not.
        if (aTp <= 0) continue;

        // Compute a'*xk
        double aTx = factor->dotProductRow(xk);

        // alpha = (b - a'*xk) / (a'*p)
        double alpha = (b - aTx) / aTp;
        if (debug) cout << "alpha: " << alpha << endl;

        // We want the minimum of all those max alphas
        if (alpha < minAlpha) {
          closestFactorIx = factorIx;
          minAlpha = alpha;
        }
      }

    }

    return std::make_pair(minAlpha, closestFactorIx);
  }

  //******************************************************************************
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues, const VectorValues& duals) const {
    InequalityFactorGraph workingSet;
    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, inequalities) {
      LinearInequality::shared_ptr workingFactor(new LinearInequality(*factor));

      double error = workingFactor->error(initialValues);
      // TODO: find a feasible initial point for LPSolver.
      // For now, we just throw an exception
      if (error > 0) throw InfeasibleInitialValues();

      if (fabs(error) < 1e-7) {
        workingFactor->activate();
      }
      else {
        workingFactor->inactivate();
      }
      workingSet.push_back(workingFactor);
    }
    return workingSet;
  }

  //******************************************************************************
  /** Optimize with the provided feasible initial values
   * TODO: throw exception if the initial values is not feasible wrt inequality constraints
   */
  pair<VectorValues, VectorValues> optimize(const VectorValues& initialValues,
      const VectorValues& duals = VectorValues()) const {

    // Initialize workingSet from the feasible initialValues
    InequalityFactorGraph workingSet = identifyActiveConstraints(
        lp_.inequalities, initialValues, duals);
    LPState state(initialValues, duals, workingSet, false, 0);

    /// main loop of the solver
    while (!state.converged) {
      state = iterate(state);
    }

    return make_pair(state.values, state.duals);
  }

  //******************************************************************************
  /**
   * Optimize without initial values
   * TODO: Find a feasible initial solution wrt inequality constraints
   */
//  pair<VectorValues, VectorValues> optimize() const {
//
//    // Initialize workingSet from the feasible initialValues
//    InequalityFactorGraph workingSet = identifyActiveConstraints(
//        lp_.inequalities, initialValues, duals);
//    LPState state(initialValues, duals, workingSet, false, 0);
//
//    /// main loop of the solver
//    while (!state.converged) {
//      state = iterate(state);
//    }
//
//    return make_pair(state.values, state.duals);
//  }

};

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
  virtual ~LPInitSolver() {};
  virtual VectorValues solve() const = 0;
};

/**
 * This LPInitSolver implements the strategy in Matlab:
 * http://www.mathworks.com/help/optim/ug/linear-programming-algorithms.html#brozyzb-9
 * Solve for x and y:
 *    min y
 *    st Ax = b
 *       Cx - y <= d
 * where y \in R, x \in R^n, and Ax = b and Cx <= d is the constraints of the original problem.
 *
 * If the solution for this problem {x*,y*} has y* <= 0, we'll have x* a feasible initial point
 * of the original problem
 * otherwise, if y* > 0 or the problem has no solution, the original problem is infeasible.
 *
 * The initial value of this initial problem can be found by solving
 *    min   ||x||^2
 *    s.t.   Ax = b
 * to have a solution x0
 * then y = max_j ( Cj*x0  - dj )  -- due to the constraints y >= Cx - d
 *
 * WARNING: If some xj in the inequality constraints does not exist in the equality constraints,
 * set them as zero for now. If that is the case, the original problem doesn't have a unique
 * solution (it could be either infeasible or unbounded).
 * So, if the initialization fails because we enforce xj=0 in the problematic
 * inequality constraint, we can't conclude that the problem is infeasible.
 * However, whether it is infeasible or unbounded, we don't have a unique solution anyway.
 */
class LPInitSolverMatlab : public LPInitSolver {
  typedef LPInitSolver Base;
public:
  LPInitSolverMatlab(const LPSolver& lpSolver) : Base(lpSolver) {}
  virtual ~LPInitSolverMatlab() {}

  virtual VectorValues solve() const  {
    // Build the graph to solve for the initial value of the initial problem
    GaussianFactorGraph::shared_ptr initOfInitGraph = buildInitOfInitGraph();
    VectorValues x0 = initOfInitGraph->optimize();
    double y0 = compute_y0(x0);
    Key yKey = maxKey(lpSolver_.keysDim()) + 1; // the unique key for y0
    VectorValues xy0(x0);
    xy0.insert(yKey, Vector::Constant(1, y0));

    // Formulate and solve the initial LP
    LP::shared_ptr initLP = buildInitialLP(yKey);

    // solve the initialLP
    LPSolver lpSolveInit(*initLP);
    VectorValues xyInit = lpSolveInit.optimize(xy0).first;
    double yOpt = xyInit.at(yKey)[0];
    xyInit.erase(yKey);
    if ( yOpt > 0)
      throw InfeasibleOrUnboundedProblem();
    else
      return xyInit;
  }

private:
  /// build initial LP
  LP::shared_ptr buildInitialLP(Key yKey) const {
    LP::shared_ptr initLP(new LP());
    initLP->cost = LinearCost(yKey, ones(1));    // min y
    initLP->equalities = lp_.equalities;         // st. Ax = b
    initLP->inequalities = addSlackVariableToInequalities(yKey, lp_.inequalities); // Cx-y <= d
    return initLP;
  }

  /// Find the max key in the problem to determine unique keys for additional slack variables
  Key maxKey(const KeyDimMap& keysDim) const {
    Key maxK = 0;
    BOOST_FOREACH(Key key, keysDim | boost::adaptors::map_keys)
      if (maxK < key)
        maxK = key;
    return maxK;
  }

  /**
   * Build the following graph to solve for an initial value of the initial problem
   *    min   ||x||^2    s.t.   Ax = b
   */
  GaussianFactorGraph::shared_ptr buildInitOfInitGraph() const {
    // first add equality constraints Ax = b
    GaussianFactorGraph::shared_ptr initGraph(new GaussianFactorGraph(lp_.equalities));

    // create factor ||x||^2 and add to the graph
    const KeyDimMap& keysDim = lpSolver_.keysDim();
    BOOST_FOREACH(Key key, keysDim | boost::adaptors::map_keys) {
      size_t dim = keysDim.at(key);
      initGraph->push_back(JacobianFactor(key, eye(dim), zero(dim)));
    }
    return initGraph;
  }

  /// y = max_j ( Cj*x0  - dj )  -- due to the inequality constraints y >= Cx - d
  double compute_y0(const VectorValues& x0) const {
    double y0 = -std::numeric_limits<double>::infinity();
    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, lp_.inequalities) {
      double error = factor->error(x0);
      if (error > y0)
        y0 = error;
    }
    return y0;
  }


  /// Collect all terms of a factor into a container. TODO: avoid memcpy?
  TermsContainer collectTerms(const LinearInequality& factor) const {
    TermsContainer terms;
    for (Factor::const_iterator it = factor.begin(); it != factor.end(); it++) {
      terms.push_back(make_pair(*it, factor.getA(it)));
    }
    return terms;
  }

  /// Turn Cx <= d into Cx - y <= d factors
  InequalityFactorGraph addSlackVariableToInequalities(Key yKey, const InequalityFactorGraph& inequalities) const {
    InequalityFactorGraph slackInequalities;
    BOOST_FOREACH(const LinearInequality::shared_ptr& factor, lp_.inequalities) {
      TermsContainer terms = collectTerms(*factor);                      // Cx
      terms.push_back(make_pair(yKey, Matrix::Constant(1, 1, -1.0)));    // -y
      double d = factor->getb()[0];
      slackInequalities.push_back(LinearInequality(terms, d, factor->dualKey()));
    }
    return slackInequalities;
  }

  // friend class for unit-testing private methods
  FRIEND_TEST(LPInitSolverMatlab, initialization);
};

} // namespace gtsam

/* ************************************************************************* */
/**
 * min -x1-x2
 * s.t.   x1 + 2x2 <= 4
 *       4x1 + 2x2 <= 12
 *       -x1 +  x2 <= 1
 *       x1, x2 >= 0
 */
LP simpleLP1() {
  LP lp;
  lp.cost = LinearCost(1, (Vector(2) << -1., -1.).finished()); // min -x1-x2 (max x1+x2)
  lp.inequalities.push_back(
      LinearInequality(1, (Vector(2) << -1, 0).finished(), 0, 1)); // x1 >= 0
  lp.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 0, -1).finished(), 0, 2)); //  x2 >= 0
  lp.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 1, 2).finished(), 4, 3)); //  x1 + 2*x2 <= 4
  lp.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 4, 2).finished(), 12, 4)); //  4x1 + 2x2 <= 12
  lp.inequalities.push_back(
      LinearInequality(1, (Vector(2) << -1, 1).finished(), 1, 5)); //  -x1 + x2 <= 1
  return lp;
}

/* ************************************************************************* */
namespace gtsam {
TEST(LPInitSolverMatlab, initialization) {
  LP lp = simpleLP1();
  LPSolver lpSolver(lp);
  LPInitSolverMatlab initSolver(lpSolver);

  GaussianFactorGraph::shared_ptr initOfInitGraph = initSolver.buildInitOfInitGraph();
  VectorValues x0 = initOfInitGraph->optimize();
  VectorValues expected_x0;
  expected_x0.insert(1, zero(2));
  CHECK(assert_equal(expected_x0, x0, 1e-10));

  double y0 = initSolver.compute_y0(x0);
  double expected_y0 = 0.0;
  DOUBLES_EQUAL(expected_y0, y0, 1e-7);

  Key yKey = 2;
  LP::shared_ptr initLP = initSolver.buildInitialLP(yKey);
  LP expectedInitLP;
  expectedInitLP.cost = LinearCost(yKey, ones(1));
  expectedInitLP.inequalities.push_back(
      LinearInequality(1, (Vector(2) << -1, 0).finished(), 2, Vector::Constant(1, -1), 0, 1)); // -x1 - y <= 0
  expectedInitLP.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 0, -1).finished(), 2, Vector::Constant(1, -1), 0, 2)); // -x2 - y <= 0
  expectedInitLP.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 1, 2).finished(), 2, Vector::Constant(1, -1), 4, 3)); //  x1 + 2*x2 - y <= 4
  expectedInitLP.inequalities.push_back(
      LinearInequality(1, (Vector(2) << 4, 2).finished(), 2, Vector::Constant(1, -1), 12, 4)); //  4x1 + 2x2 - y <= 12
  expectedInitLP.inequalities.push_back(
      LinearInequality(1, (Vector(2) << -1, 1).finished(), 2, Vector::Constant(1, -1), 1, 5)); //  -x1 + x2 - y <= 1
  CHECK(assert_equal(expectedInitLP, *initLP, 1e-10));

  LPSolver lpSolveInit(*initLP);
  VectorValues xy0(x0);
  xy0.insert(yKey, Vector::Constant(1, y0));
  VectorValues xyInit = lpSolveInit.optimize(xy0).first;
  VectorValues expected_init;
  expected_init.insert(1, (Vector(2) << 1, 1).finished());
  expected_init.insert(2, Vector::Constant(1, -1));
  CHECK(assert_equal(expected_init, xyInit, 1e-10));

  VectorValues x = initSolver.solve();
  CHECK(lp.isFeasible(x));
}
}

/* ************************************************************************* */
/**
 * TEST gtsam solver with an over-constrained system
 *  x + y = 1
 *  x - y = 5
 *  x + 2y = 6
 */
TEST(LPSolver, overConstrainedLinearSystem) {
  GaussianFactorGraph graph;
  Matrix A1 = (Matrix(3,1) <<1,1,1).finished();
  Matrix A2 = (Matrix(3,1) <<1,-1,2).finished();
  Vector b = (Vector(3) << 1, 5, 6).finished();
  JacobianFactor factor(1, A1, 2, A2, b, noiseModel::Constrained::All(3));
  graph.push_back(factor);

  VectorValues x = graph.optimize();
  // This check confirms that gtsam linear constraint solver can't handle over-constrained system
  CHECK(factor.error(x) != 0.0);
}

TEST(LPSolver, overConstrainedLinearSystem2) {
  GaussianFactorGraph graph;
  graph.push_back(JacobianFactor(1, ones(1, 1), 2, ones(1, 1), ones(1), noiseModel::Constrained::All(1)));
  graph.push_back(JacobianFactor(1, ones(1, 1), 2, -ones(1, 1), 5*ones(1), noiseModel::Constrained::All(1)));
  graph.push_back(JacobianFactor(1, ones(1, 1), 2, 2*ones(1, 1), 6*ones(1), noiseModel::Constrained::All(1)));
  VectorValues x = graph.optimize();
  // This check confirms that gtsam linear constraint solver can't handle over-constrained system
  CHECK(graph.error(x) != 0.0);
}

/* ************************************************************************* */
TEST(LPSolver, simpleTest1) {
  LP lp = simpleLP1();

  LPSolver lpSolver(lp);
  VectorValues init;
  init.insert(1, zero(2));

  VectorValues x1 = lpSolver.solveWithCurrentWorkingSet(init,
      InequalityFactorGraph());
  VectorValues expected_x1;
  expected_x1.insert(1, (Vector(2) << 1, 1).finished());
  CHECK(assert_equal(expected_x1, x1, 1e-10));

  VectorValues result, duals;
  boost::tie(result, duals) = lpSolver.optimize(init);
  VectorValues expectedResult;
  expectedResult.insert(1, (Vector(2)<<8./3., 2./3.).finished());
  CHECK(assert_equal(expectedResult, result, 1e-10));
}

/**
 * TODO: More TEST cases:
 * - Infeasible
 * - Unbounded
 * - Underdetermined
 */
/* ************************************************************************* */
TEST(LPSolver, LinearCost) {
  LinearCost cost(1, (Vector(3) << 2., 4., 6.).finished());
  VectorValues x;
  x.insert(1, (Vector(3) << 1., 3., 5.).finished());
  double error = cost.error(x);
  double expectedError = 44.0;
  DOUBLES_EQUAL(expectedError, error, 1e-100);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

