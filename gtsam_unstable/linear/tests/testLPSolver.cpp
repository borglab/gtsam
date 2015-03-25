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

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

/* ************************************************************************* */
/** An exception indicating that the noise model dimension passed into a
 * JacobianFactor has a different dimensionality than the factor. */
class InfeasibleInitialValues: public ThreadsafeException<
    InfeasibleInitialValues> {
public:
  InfeasibleInitialValues() {
  }
  virtual ~InfeasibleInitialValues() throw () {
  }

  virtual const char* what() const throw () {
    if (description_.empty()) description_ =
        "An infeasible intial value was provided for the QPSolver.\n"
            "This current version of QPSolver does not handle infeasible"
            "initial point due to the lack of a LPSolver.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};

struct LP {
  LinearCost cost; //!< Linear cost factor
  EqualityFactorGraph equalities; //!< Linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< Linear inequality constraints: cI(x) <= 0

  void print(const string& s = "") const {
    std::cout << s << std::endl;
    cost.print("Linear cost: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }
};

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

class LPSolver {
  const LP& lp_; //!< the linear programming problem
  GaussianFactorGraph baseGraph_; //!< unchanged factors needed in every iteration
  VariableIndex costVariableIndex_, equalityVariableIndex_,
      inequalityVariableIndex_; //!< index to corresponding factors to build dual graphs
  FastSet<Key> constrainedKeys_; //!< all constrained keys, will become factors in dual graphs

public:
  LPSolver(const LP& lp) :
      lp_(lp) {
    // Push back factors that are the same in every iteration to the base graph.
    // Those include the equality constraints and zero priors for keys that are not
    // in the cost
    baseGraph_.push_back(lp_.equalities);
    baseGraph_.push_back(*createZeroPriors(lp_.cost.keys(), lp_.equalities));
    baseGraph_.push_back(*createZeroPriors(lp_.cost.keys(), lp_.inequalities));
    equalityVariableIndex_ = VariableIndex(lp_.equalities);
    inequalityVariableIndex_ = VariableIndex(lp_.inequalities);
    constrainedKeys_ = lp_.equalities.keys();
    constrainedKeys_.merge(lp_.inequalities.keys());
  }

  //******************************************************************************
  /**
   * Create a zero prior for any keys in the graph that don't exist in the cost
   */
  template<class LinearGraph>
  GaussianFactorGraph::shared_ptr createZeroPriors(const KeyVector& costKeys,
      const LinearGraph& linearGraph) const {
    GaussianFactorGraph::shared_ptr graph(new GaussianFactorGraph());
    BOOST_FOREACH(const typename LinearGraph::sharedFactor& factor, linearGraph) {
      if (!factor) continue;
      BOOST_FOREACH(Key key, factor->keys()) {
        if (find(costKeys.begin(), costKeys.end(), key) == costKeys.end()) {
          size_t dim = factor->getDim(factor->find(key));
          graph->push_back(JacobianFactor(key, eye(dim), zero(dim)));
        }
      }
    }
    return graph;
  }

  //******************************************************************************
  LPState iterate(const LPState& state) const {
    static bool debug = true;

    // Solve with the current working set
    // LP: project the objective neggradient to the constraint's null space
    // to find the direction to move
    VectorValues newValues = solveWithCurrentWorkingSet(state.values,
        state.workingSet);
//    if (debug) state.workingSet.print("Working set:");
    if (debug) (newValues - state.values).print("New direction:");

    // If we CAN'T move further
    // LP: projection on nullspace is zero: we are at a vertex
    if (newValues.equals(state.values, 1e-7)) {
      // If we still have equality constraints: the problem is over-constrained. No solution!
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
   * is the projection of the gradient onto the constraint subspace
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
  std::vector<std::pair<Key, Matrix> > collectDualJacobians(Key key,
      const FactorGraph<FACTOR>& graph,
      const VariableIndex& variableIndex) const {
    std::vector<std::pair<Key, Matrix> > Aterms;
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
    std::vector<std::pair<Key, Matrix> > Aterms = collectDualJacobians<
        LinearEquality>(key, lp_.equalities, equalityVariableIndex_);
    std::vector<std::pair<Key, Matrix> > AtermsInequalities =
        collectDualJacobians<LinearInequality>(key, workingSet,
            inequalityVariableIndex_);
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

};

/* ************************************************************************* */
TEST(LPSolver, simpleTest1) {
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

  LPSolver lpSolver(lp);
  VectorValues init;
  init.insert(1, zero(2));

  VectorValues x1 = lpSolver.solveWithCurrentWorkingSet(init,
      InequalityFactorGraph());
  x1.print("x1: ");

  VectorValues result, duals;
  boost::tie(result, duals) = lpSolver.optimize(init);
  VectorValues expectedResult;
  expectedResult.insert(1, (Vector(2)<<8./3., 2./3.).finished());
  CHECK(assert_equal(expectedResult, result, 1e-10));
}

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

