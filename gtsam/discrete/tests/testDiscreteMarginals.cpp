/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @ file testDiscreteMarginals.cpp
 * @date Feb 14, 2011
 * @author Abhijit Kundu
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteMarginals.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

TEST( DiscreteMarginals, UGM_small ) {
  size_t nrStates = 2;
  DiscreteKey Cathy(1, nrStates), Heather(2, nrStates), Mark(3, nrStates),
      Allison(4, nrStates);
  DiscreteFactorGraph graph;

  // add node potentials
  graph.add(Cathy, "1 3");
  graph.add(Heather, "9 1");
  graph.add(Mark, "1 3");
  graph.add(Allison, "9 1");

  // add edge potentials
  graph.add(Cathy & Heather, "2 1 1 2");
  graph.add(Heather & Mark, "2 1 1 2");
  graph.add(Mark & Allison, "2 1 1 2");

  DiscreteMarginals marginals(graph);

  DiscreteFactor::shared_ptr actualC = marginals(Cathy.first);
  DiscreteFactor::Values values;
  values[Cathy.first] = 0;
  EXPECT_DOUBLES_EQUAL( 1.944, (*actualC)(values), 1e-9);

  Vector actualCvector = marginals.marginalProbabilities(Cathy.first);
  EXPECT(assert_equal(Vector_(2,0.7,0.3), actualCvector, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

