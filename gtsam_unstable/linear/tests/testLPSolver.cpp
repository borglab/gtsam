/*
 * testLPSolver.cpp
 * @brief:
 * @date: May 1, 2014
 * @author: Duy-Nguyen Ta
 */


#include <CppUnitLite/TestHarness.h>
#include <boost/range/adaptor/map.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/LPSolver.h>
#include <gtsam_unstable/linear/LinearInequalityFactorGraph.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

TEST(LPSolver, oneD) {
  VectorValues objCoeffs;
  objCoeffs.insert(Y(1), repeat(1, -5.0));
  objCoeffs.insert(X(2), repeat(1, -4.0));
  objCoeffs.insert(X(3), repeat(1, -6.0));
  LinearInequality inequality(
      Y(1), (Matrix(3,1)<< 1,3,3),
      X(2), (Matrix(3,1)<< -1,2,2),
      X(3), (Matrix(3,1)<< 1,4,0), (Vector(3)<<20,42,30), 0);
  VectorValues lowerBounds;
  lowerBounds.insert(Y(1), zero(1));
  lowerBounds.insert(X(2), zero(1));
  lowerBounds.insert(X(3), zero(1));
  LinearInequalityFactorGraph::shared_ptr inequalities(new LinearInequalityFactorGraph);
  inequalities->push_back(inequality);
  LinearEqualityFactorGraph::shared_ptr equalities(new LinearEqualityFactorGraph);

  LPSolver solver(objCoeffs, equalities, inequalities, lowerBounds);
  vector<int> columnNo = solver.buildColumnNo(objCoeffs | boost::adaptors::map_keys);
  LONGS_EQUAL(3, columnNo.size());
  LONGS_EQUAL(1, columnNo[0]);
  LONGS_EQUAL(2, columnNo[1]);
  LONGS_EQUAL(3, columnNo[2]);

  std::map<Key, size_t> variableColumnNo = solver.variableColumnNo(),
                        variableDims = solver.variableDims();
  LONGS_EQUAL(3, variableColumnNo.size());
//  LONGS_EQUAL(1, variableColumnNo.at(Y(1)));
//  LONGS_EQUAL(2, variableColumnNo.at(X(2)));
//  LONGS_EQUAL(3, variableColumnNo.at(X(3)));
  BOOST_FOREACH(Key key, variableDims | boost::adaptors::map_keys) {
    LONGS_EQUAL(1, variableDims.at(key));
  }
  size_t nrColumns = solver.nrColumns();
  LONGS_EQUAL(3, nrColumns);
  KeySet freeVars = solver.freeVars();
  LONGS_EQUAL(0, freeVars.size());

  VectorValues solution = solver.solve();
  VectorValues expectedSolution;
  expectedSolution.insert(Y(1), zero(1));
  expectedSolution.insert(X(2), 15*ones(1));
  expectedSolution.insert(X(3), 3*ones(1));
  EXPECT(assert_equal(expectedSolution, solution));
}

TEST(LPSolver, threeD) {
  VectorValues objCoeffs;
  objCoeffs.insert(X(1), (Vector(3)<<-5.0, -4.0, -6.0));
  LinearInequality inequality(
      X(1), (Matrix(3,3)<< 1,-1,1,3,2,4,3,2,0), (Vector(3)<<20,42,30), 0);
  VectorValues lowerBounds;
  lowerBounds.insert(X(1), zeros(3,1));

  LinearInequalityFactorGraph::shared_ptr inequalities(new LinearInequalityFactorGraph);
  inequalities->push_back(inequality);
  LinearEqualityFactorGraph::shared_ptr equalities(new LinearEqualityFactorGraph);

  LPSolver solver(objCoeffs, equalities, inequalities, lowerBounds);
  vector<int> columnNo = solver.buildColumnNo(objCoeffs | boost::adaptors::map_keys);
  LONGS_EQUAL(3, columnNo.size());
  LONGS_EQUAL(1, columnNo[0]);
  LONGS_EQUAL(2, columnNo[1]);
  LONGS_EQUAL(3, columnNo[2]);

  std::map<Key, size_t> variableColumnNo = solver.variableColumnNo(),
                        variableDims = solver.variableDims();
  LONGS_EQUAL(1, variableColumnNo.size());
  LONGS_EQUAL(1, variableColumnNo.at(X(1)));
  BOOST_FOREACH(Key key, variableDims | boost::adaptors::map_keys) {
    LONGS_EQUAL(3, variableDims.at(key));
  }
  size_t nrColumns = solver.nrColumns();
  LONGS_EQUAL(3, nrColumns);
  KeySet freeVars = solver.freeVars();
  LONGS_EQUAL(0, freeVars.size());

  VectorValues solution = solver.solve();
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(3)<<0.0, 15, 3));
  EXPECT(assert_equal(expectedSolution, solution));
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
