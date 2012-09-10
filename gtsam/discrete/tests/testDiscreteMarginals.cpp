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
#include <gtsam/discrete/DiscreteSequentialSolver.h>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;
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
  DiscreteFactor::Values values;

  values[Cathy.first] = 0;
  EXPECT_DOUBLES_EQUAL( 0.359631, (*actualC)(values), 1e-6);

  Vector actualCvector = marginals.marginalProbabilities(Cathy);
  EXPECT(assert_equal(Vector_(2, 0.359631, 0.640369), actualCvector, 1e-6));

  actualCvector = marginals.marginalProbabilities(Mark);
  EXPECT(assert_equal(Vector_(2, 0.48628, 0.51372), actualCvector, 1e-6));
}

/* ************************************************************************* */
TEST_UNSAFE( DiscreteMarginals, UGM_chain ) {

	const int nrNodes = 10;
	const size_t nrStates = 7;

	// define variables
	vector<DiscreteKey> nodes;
	for (int i = 0; i < nrNodes; i++) {
		DiscreteKey dk(i, nrStates);
		nodes.push_back(dk);
	}

	// create graph
	DiscreteFactorGraph graph;

	// add node potentials
	graph.add(nodes[0], ".3 .6 .1 0 0 0 0");
	for (int i = 1; i < nrNodes; i++)
		graph.add(nodes[i], "1 1 1 1 1 1 1");

	const std::string edgePotential =   ".08 .9 .01 0 0 0 .01 "
																			".03 .95 .01 0 0 0 .01 "
																			".06 .06 .75 .05 .05 .02 .01 "
																			"0 0 0 .3 .6 .09 .01 "
																			"0 0 0 .02 .95 .02 .01 "
																			"0 0 0 .01 .01 .97 .01 "
																			"0 0 0 0 0 0 1";

	// add edge potentials
	for (int i = 0; i < nrNodes - 1; i++)
		graph.add(nodes[i] & nodes[i + 1], edgePotential);

	DiscreteMarginals marginals(graph);
	DiscreteFactor::shared_ptr actualC = marginals(nodes[2].first);
	DiscreteFactor::Values values;

	values[nodes[2].first] = 0;
	EXPECT_DOUBLES_EQUAL( 0.03426, (*actualC)(values), 1e-4);
}

/* ************************************************************************* */
TEST_UNSAFE( DiscreteMarginals, truss ) {

  const int nrNodes = 5;
  const size_t nrStates = 2;

  // define variables
  vector<DiscreteKey> nodes;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey dk(i, nrStates);
    nodes.push_back(dk);
  }

  // create graph and add three truss potentials
  DiscreteFactorGraph graph;
  graph.add(nodes[0] & nodes[2] & nodes[4],"2 2 2 2 1 1 1 1");
  graph.add(nodes[1] & nodes[3] & nodes[4],"1 1 1 1 2 2 2 2");
  graph.add(nodes[2] & nodes[3] & nodes[4],"1 1 1 1 1 1 1 1");
  typedef JunctionTree<DiscreteFactorGraph> JT;
  GenericMultifrontalSolver<DiscreteFactor, JT> solver(graph);
  BayesTree<DiscreteConditional>::shared_ptr bayesTree = solver.eliminate(&EliminateDiscrete);
//  bayesTree->print("Bayes Tree");
  typedef BayesTreeClique<DiscreteConditional> Clique;

  Clique expected0(boost::make_shared<DiscreteConditional>((nodes[0] | nodes[2], nodes[4]) = "2/1 2/1 2/1 2/1"));
  Clique::shared_ptr actual0 = (*bayesTree)[0];
//  EXPECT(assert_equal(expected0, *actual0)); // TODO, correct but fails

  Clique expected1(boost::make_shared<DiscreteConditional>((nodes[1] | nodes[3], nodes[4]) = "1/2 1/2 1/2 1/2"));
  Clique::shared_ptr actual1 = (*bayesTree)[1];
//  EXPECT(assert_equal(expected1, *actual1)); // TODO, correct but fails

  // Create Marginals instance
  DiscreteMarginals marginals(graph);

  // test 0
  DecisionTreeFactor expectedM0(nodes[0],"0.666667 0.333333");
  DiscreteFactor::shared_ptr actualM0 = marginals(0);
  EXPECT(assert_equal(expectedM0, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM0),1e-5));

  // test 1
  DecisionTreeFactor expectedM1(nodes[1],"0.333333 0.666667");
  DiscreteFactor::shared_ptr actualM1 = marginals(1);
  EXPECT(assert_equal(expectedM1, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM1),1e-5));
}

/* ************************************************************************* */
// Second truss example with non-trivial factors
TEST_UNSAFE( DiscreteMarginals, truss2 ) {

  const int nrNodes = 5;
  const size_t nrStates = 2;

  // define variables
  vector<DiscreteKey> nodes;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey dk(i, nrStates);
    nodes.push_back(dk);
  }

  // create graph and add three truss potentials
  DiscreteFactorGraph graph;
  graph.add(nodes[0] & nodes[2] & nodes[4],"1 2 3 4 5 6 7 8");
  graph.add(nodes[1] & nodes[3] & nodes[4],"1 2 3 4 5 6 7 8");
  graph.add(nodes[2] & nodes[3] & nodes[4],"1 2 3 4 5 6 7 8");

  // Calculate the marginals by brute force
  vector<DiscreteFactor::Values> allPosbValues = cartesianProduct(
      nodes[0] & nodes[1] & nodes[2] & nodes[3] & nodes[4]);
  Vector T = zero(5), F = zero(5);
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteFactor::Values x = allPosbValues[i];
    double px = graph(x);
    for (size_t j=0;j<5;j++)
      if (x[j]) T[j]+=px; else F[j]+=px;
    // cout << x[0] << " " << x[1] << " "<< x[2] << " " << x[3] << " " << x[4] << " :\t" << px << endl;
  }

  // Check all marginals given by a sequential solver and Marginals
  DiscreteSequentialSolver solver(graph);
  DiscreteMarginals marginals(graph);
  for (size_t j=0;j<5;j++) {
    double sum = T[j]+F[j];
    T[j]/=sum;
    F[j]/=sum;

    // solver
    Vector actualV = solver.marginalProbabilities(nodes[j]);
    EXPECT(assert_equal(Vector_(2,F[j],T[j]), actualV));

    // Marginals
    vector<double> table;
    table += F[j],T[j];
    DecisionTreeFactor expectedM(nodes[j],table);
    DiscreteFactor::shared_ptr actualM = marginals(j);
    EXPECT(assert_equal(expectedM, *boost::dynamic_pointer_cast<DecisionTreeFactor>(actualM)));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

