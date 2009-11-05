/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesNet.h"
#include "GaussianBayesNet.h"
#include "Ordering.h"
#include "BayesTree-inl.h"
#include "smallExample.h"

using namespace gtsam;

// Conditionals for ASIA example from the tutorial with A and D evidence
SymbolicConditional::shared_ptr B(new SymbolicConditional("B")), L(
		new SymbolicConditional("L", "B")), E(
		new SymbolicConditional("E", "L", "B")), S(new SymbolicConditional("S",
		"L", "B")), T(new SymbolicConditional("T", "E", "L")), X(
		new SymbolicConditional("X", "E"));

/* ************************************************************************* */
TEST( BayesTree, Front )
{
	SymbolicBayesNet f1;
	f1.push_back(B);
	f1.push_back(L);
	SymbolicBayesNet f2;
	f2.push_back(L);
	f2.push_back(B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	// Create using insert
	BayesTree<SymbolicConditional> bayesTree;
	bayesTree.insert(B);
	bayesTree.insert(L);
	bayesTree.insert(E);
	bayesTree.insert(S);
	bayesTree.insert(T);
	bayesTree.insert(X);

	// Check Size
	LONGS_EQUAL(6,bayesTree.size());

	// Check root
	BayesNet<SymbolicConditional> expected_root;
	expected_root.push_back(E);
	expected_root.push_back(L);
	expected_root.push_back(B);
	boost::shared_ptr<BayesNet<SymbolicConditional> > actual_root = bayesTree.root();
	CHECK(assert_equal(expected_root,*actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	SymbolicBayesNet ASIA;
	ASIA.push_back(X);
	ASIA.push_back(T);
	ASIA.push_back(S);
	ASIA.push_back(E);
	ASIA.push_back(L);
	ASIA.push_back(B);
	BayesTree<SymbolicConditional> bayesTree2(ASIA);
	//bayesTree2.print("bayesTree2");

	// Check whether the same
	CHECK(assert_equal(bayesTree,bayesTree2));
}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
 x6 x7
   x5 : x6
     x4 : x5
       x3 : x4
         x2 : x3
           x1 : x2
/* ************************************************************************* */
TEST( BayesTree, smoother )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	for (int t = 1; t <= 7; t++)
		ordering.push_back(symbol('x', t));

	// eliminate using the "natural" ordering
	GaussianBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);

	// Create the Bayes tree
	BayesTree<ConditionalGaussian> bayesTree(*chordalBayesNet);
	LONGS_EQUAL(7,bayesTree.size());
}

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
 x5 x6 x4
   x3 x2 : x4
     x1 : x2
   x7 : x6
/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_marginals )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	GaussianBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);

	// Create the Bayes tree
	BayesTree<ConditionalGaussian> bayesTree(*chordalBayesNet);
	LONGS_EQUAL(7,bayesTree.size());

	// Check root clique
	//BayesNet<ConditionalGaussian> expected_root;
	//BayesNet<ConditionalGaussian> actual_root = bayesTree.root();
	//CHECK(assert_equal(expected_root,actual_root));

	// Check marginal on x1
  double data1[] = { 1.0, 0.0,
                     0.0, 1.0};
  Matrix R1 = Matrix_(2,2, data1);
  Vector d1(2); d1(0) = -0.615385; d1(1) = 0;
  Vector tau1(2); tau1(0) = 1.61803; tau1(1) = 1.61803;
	ConditionalGaussian expected("x1",d1, R1, tau1);
	ConditionalGaussian::shared_ptr actual = bayesTree.marginal<LinearFactor>("x1");
	CHECK(assert_equal(expected,*actual,1e-4));

	// JunctionTree is an undirected tree of cliques
	// JunctionTree<ConditionalGaussian> marginals = bayesTree.marginals();
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
