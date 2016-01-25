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
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

#include <gtsam_unstable/linear/LP.h>
#include <gtsam_unstable/linear/LPState.h>
#include <gtsam_unstable/linear/LPSolver.h>
#include <gtsam_unstable/linear/InfeasibleOrUnboundedProblem.h>
#include <gtsam_unstable/linear/LPInitSolver.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

namespace gtsam {

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

