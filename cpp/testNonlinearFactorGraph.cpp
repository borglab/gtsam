/** 
 * @file    testNonlinearFactorGraph.cpp
 * @brief   Unit tests for Non-Linear Factor Graph
 * @brief   testNonlinearFactorGraph
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */


/*STL/C++*/
#include <iostream>
using namespace std;

#include <CppUnitLite/TestHarness.h>

#include "Matrix.h"
#include "smallExample.h"

using namespace gtsam;

/* ************************************************************************* */
TEST( NonlinearFactorGraph, equals ){

  NonlinearFactorGraph fg  = createNonlinearFactorGraph();
  NonlinearFactorGraph fg2 = createNonlinearFactorGraph();
  CHECK( fg.equals(fg2) );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, error )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();

  FGConfig c1 = createConfig();
  double actual1 = fg.error(c1);
  DOUBLES_EQUAL( 0.0, actual1, 1e-9 );

  FGConfig c2 = createNoisyConfig();
  double actual2 = fg.error(c2);
  DOUBLES_EQUAL( 5.625, actual2, 1e-9 );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, probPrime )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  FGConfig cfg = createConfig();

  // evaluate the probability of the factor graph
  double actual = fg.probPrime(cfg);
  double expected = 1.0;
  DOUBLES_EQUAL(expected,actual,0);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, linearize )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  FGConfig initial = createNoisyConfig();
  LinearFactorGraph linearized = fg.linearize(initial);
  LinearFactorGraph expected = createLinearFactorGraph();
  CHECK(expected.equals(linearized));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, iterate )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  FGConfig initial = createNoisyConfig();

  // Expected configuration is the difference between the noisy config
  // and the ground-truth config. One step only because it's linear !
  FGConfig expected;
  Vector dl1(2); dl1(0)=-0.1; dl1(1)= 0.1; expected.insert("l1",dl1);
  Vector dx1(2); dx1(0)=-0.1; dx1(1)=-0.1; expected.insert("x1",dx1);
  Vector dx2(2); dx2(0)= 0.1; dx2(1)=-0.2; expected.insert("x2",dx2);

  // Check one ordering
  Ordering ord1;
  ord1.push_back("x2");
  ord1.push_back("l1");
  ord1.push_back("x1");
  FGConfig actual1 = fg.iterate(initial, ord1);
  CHECK(actual1.equals(expected));

  // Check another
  Ordering ord2;
  ord2.push_back("x1");
  ord2.push_back("x2");
  ord2.push_back("l1");
  FGConfig actual2 = fg.iterate(initial, ord2);
  CHECK(actual2.equals(expected));

  // And yet another...
  Ordering ord3;
  ord3.push_back("l1");
  ord3.push_back("x1");
  ord3.push_back("x2");
  FGConfig actual3 = fg.iterate(initial, ord3);
  CHECK(actual3.equals(expected));

}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, optimize ) {

  NonlinearFactorGraph fg = createReallyNonlinearFactorGraph();

  // test error at minimum
  Vector xstar = Vector_(1,0.0);
  FGConfig cstar;
  cstar.insert("x",xstar);
  DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

  // test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 = 
  Vector x0 = Vector_(1,3.0);
  FGConfig c0;
  c0.insert("x",x0);
  DOUBLES_EQUAL(199.0,fg.error(c0),1e-3);

  // optimize parameters
  Ordering ord;
  ord.push_back("x");
  double relativeErrorTreshold = 1e-5;
  double absoluteErrorTreshold = 1e-5;

  // Gauss-Newton
  FGConfig actual = c0;
  fg.optimize(actual,ord,relativeErrorTreshold,absoluteErrorTreshold);
  CHECK(actual.equals(cstar));

  // Levenberg-Marquardt
  FGConfig actual2 = c0;
  double lambda0 = 1000, lambdaFactor = 10;
  fg.optimizeLM(actual2,ord,relativeErrorTreshold,absoluteErrorTreshold,0,lambda0,lambdaFactor);
  CHECK(actual2.equals(cstar));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, iterateLM ) {

  // really non-linear factor graph
  NonlinearFactorGraph fg = createReallyNonlinearFactorGraph();

  // config far from minimum
  Vector x0 = Vector_(1,3.0);
  FGConfig config;
  config.insert("x",x0);

  // ordering
  Ordering ord;
  ord.push_back("x");

  // normal iterate
  int verbosity = 0;
  FGConfig actual1 = config;
  fg.iterate(actual1,ord,verbosity);

  // LM iterate with lambda 0 should be the same
  FGConfig actual2 = config;
  double lambda0 = 0, lambdaFactor = 10;
  double currentError = fg.error(actual2);
  fg.iterateLM(actual2, currentError, lambda0, lambdaFactor, ord, verbosity);

  CHECK(actual1.equals(actual2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
