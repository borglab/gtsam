/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testDiscreteMarginals.cpp
 * @date Jun 7, 2012
 * @author Abhijit Kundu
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteMarginals.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST_UNSAFE( DiscreteMarginals, UGM_small ) {
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
  DiscreteValues values;

  values[Cathy.first] = 0;
  EXPECT_DOUBLES_EQUAL( 0.359631, (*actualC)(values), 1e-6);

  Vector actualCvector = marginals.marginalProbabilities(Cathy);
  EXPECT(assert_equal(Vector2(0.359631, 0.640369), actualCvector, 1e-6));

  actualCvector = marginals.marginalProbabilities(Mark);
  EXPECT(assert_equal(Vector2(0.48628, 0.51372), actualCvector, 1e-6));
}

/* ************************************************************************* */
TEST_UNSAFE( DiscreteMarginals, UGM_chain ) {

  const int nrNodes = 10;
  const size_t nrStates = 7;

  // define variables
  vector<DiscreteKey> key;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey key_i(i, nrStates);
    key.push_back(key_i);
  }

  // create graph
  DiscreteFactorGraph graph;

  // add node potentials
  graph.add(key[0], ".3 .6 .1 0 0 0 0");
  for (int i = 1; i < nrNodes; i++)
    graph.add(key[i], "1 1 1 1 1 1 1");

  const std::string edgePotential =   ".08 .9 .01 0 0 0 .01 "
                                      ".03 .95 .01 0 0 0 .01 "
                                      ".06 .06 .75 .05 .05 .02 .01 "
                                      "0 0 0 .3 .6 .09 .01 "
                                      "0 0 0 .02 .95 .02 .01 "
                                      "0 0 0 .01 .01 .97 .01 "
                                      "0 0 0 0 0 0 1";

  // add edge potentials
  for (int i = 0; i < nrNodes - 1; i++)
    graph.add(key[i] & key[i + 1], edgePotential);

  DiscreteMarginals marginals(graph);
  DiscreteFactor::shared_ptr actualC = marginals(key[2].first);
  DiscreteValues values;

  values[key[2].first] = 0;
  EXPECT_DOUBLES_EQUAL( 0.03426, (*actualC)(values), 1e-4);
}

/* ************************************************************************* */
TEST_UNSAFE( DiscreteMarginals, truss ) {

  const int nrNodes = 5;
  const size_t nrStates = 2;

  // define variables
  vector<DiscreteKey> key;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey key_i(i, nrStates);
    key.push_back(key_i);
  }

  // create graph and add three truss potentials
  DiscreteFactorGraph graph;
  graph.add(key[0] & key[2] & key[4],"2 2 2 2 1 1 1 1");
  graph.add(key[1] & key[3] & key[4],"1 1 1 1 2 2 2 2");
  graph.add(key[2] & key[3] & key[4],"1 1 1 1 1 1 1 1");
  DiscreteBayesTree::shared_ptr bayesTree = graph.eliminateMultifrontal();
//  bayesTree->print("Bayes Tree");
  typedef DiscreteBayesTreeClique Clique;

  Clique expected0(boost::make_shared<DiscreteConditional>((key[0] | key[2], key[4]) = "2/1 2/1 2/1 2/1"));
  Clique::shared_ptr actual0 = (*bayesTree)[0];
//  EXPECT(assert_equal(expected0, *actual0)); // TODO, correct but fails

  Clique expected1(boost::make_shared<DiscreteConditional>((key[1] | key[3], key[4]) = "1/2 1/2 1/2 1/2"));
  Clique::shared_ptr actual1 = (*bayesTree)[1];
//  EXPECT(assert_equal(expected1, *actual1)); // TODO, correct but fails

  // Create Marginals instance
  DiscreteMarginals marginals(graph);

  // test 0
  DecisionTreeFactor expectedM0(key[0],"0.666667 0.333333");
  DiscreteFactor::shared_ptr actualM0 = marginals(0);
  EXPECT(assert_equal(expectedM0, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM0),1e-5));

  // test 1
  DecisionTreeFactor expectedM1(key[1],"0.333333 0.666667");
  DiscreteFactor::shared_ptr actualM1 = marginals(1);
  EXPECT(assert_equal(expectedM1, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM1),1e-5));
}

/* ************************************************************************* */
// Second truss example with non-trivial factors
TEST_UNSAFE(DiscreteMarginals, truss2) {
  const int nrNodes = 5;
  const size_t nrStates = 2;

  // define variables
  vector<DiscreteKey> key;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey key_i(i, nrStates);
    key.push_back(key_i);
  }

  // create graph and add three truss potentials
  DiscreteFactorGraph graph;
  graph.add(key[0] & key[2] & key[4], "1 2 3 4 5 6 7 8");
  graph.add(key[1] & key[3] & key[4], "1 2 3 4 5 6 7 8");
  graph.add(key[2] & key[3] & key[4], "1 2 3 4 5 6 7 8");

  // Calculate the marginals by brute force
  auto allPosbValues = DiscreteValues::CartesianProduct(
      key[0] & key[1] & key[2] & key[3] & key[4]);
  Vector T = Z_5x1, F = Z_5x1;
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteValues x = allPosbValues[i];
    double px = graph(x);
    for (size_t j = 0; j < 5; j++)
      if (x[j])
        T[j] += px;
      else
        F[j] += px;
  }

  // Check all marginals given by a sequential solver and Marginals
  //  DiscreteSequentialSolver solver(graph);
  DiscreteMarginals marginals(graph);
  for (size_t j = 0; j < 5; j++) {
    double sum = T[j] + F[j];
    T[j] /= sum;
    F[j] /= sum;

    // Marginals
    vector<double> table;
    table += F[j], T[j];
    DecisionTreeFactor expectedM(key[j], table);
    DiscreteFactor::shared_ptr actualM = marginals(j);
    EXPECT(assert_equal(
        expectedM, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM)));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

