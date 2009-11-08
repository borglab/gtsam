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

typedef BayesTree<ConditionalGaussian> Gaussian;

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
C1 x6 x7
C2   x5 : x6
C3     x4 : x5
C4       x3 : x4
C5         x2 : x3
C6           x1 : x2
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
	Gaussian bayesTree(*chordalBayesNet);
	LONGS_EQUAL(7,bayesTree.size());

	// Check the conditional P(Root|Root)
	BayesNet<ConditionalGaussian> empty;
	Gaussian::sharedClique R = bayesTree.root();
	Gaussian::sharedBayesNet actual1 = R->shortcut<LinearFactor>(R);
	CHECK(assert_equal(empty,*actual1,1e-4));

	// Check the conditional P(C2|Root)
	Gaussian::sharedClique C2 = bayesTree["x5"];
	Gaussian::sharedBayesNet actual2 = C2->shortcut<LinearFactor>(R);
	CHECK(assert_equal(empty,*actual2,1e-4));

	// Check the conditional P(C3|Root)
  Vector sigma3 = repeat(2, 0.61808);
  Matrix A56 = Matrix_(2,2,-0.382022,0.,0.,-0.382022);
	ConditionalGaussian::shared_ptr cg3(new ConditionalGaussian("x5", zero(2), eye(2), "x6", A56, sigma3));
	BayesNet<ConditionalGaussian> expected3; expected3.push_back(cg3);
	Gaussian::sharedClique C3 = bayesTree["x4"];
	Gaussian::sharedBayesNet actual3 = C3->shortcut<LinearFactor>(R);
	CHECK(assert_equal(expected3,*actual3,1e-4));

	// Check the conditional P(C4|Root)
  Vector sigma4 = repeat(2, 0.661968);
  Matrix A46 = Matrix_(2,2,-0.146067,0.,0.,-0.146067);
	ConditionalGaussian::shared_ptr cg4(new ConditionalGaussian("x4", zero(2), eye(2), "x6", A46, sigma4));
	BayesNet<ConditionalGaussian> expected4; expected4.push_back(cg4);
	Gaussian::sharedClique C4 = bayesTree["x3"];
	Gaussian::sharedBayesNet actual4 = C4->shortcut<LinearFactor>(R);
	CHECK(assert_equal(expected4,*actual4,1e-4));
}

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:

	 Node[x1] P(x1 | x2)
	 Node[x3] P(x3 | x2 x4)
	 Node[x5] P(x5 | x4 x6)
	 Node[x7] P(x7 | x6)
	 Node[x2] P(x2 | x4)
	 Node[x6] P(x6 | x4)
	 Node[x4] P(x4)

 becomes

	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_marginals )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	GaussianBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);

//  SymbolicBayesNet symbolic(*chordalBayesNet);
//  symbolic.print("chordalBayesNet");

  VectorConfig expectedSolution;
  Vector delta = zero(2);
  BOOST_FOREACH(string key, ordering)
		expectedSolution.insert(key,delta);
  boost::shared_ptr<VectorConfig> actualSolution = chordalBayesNet->optimize();
	CHECK(assert_equal(expectedSolution,*actualSolution,1e-4));

	// Create the Bayes tree
	Gaussian bayesTree(*chordalBayesNet);
	LONGS_EQUAL(7,bayesTree.size());

	// Marginals

	// Check marginal on x1
  GaussianBayesNet expected1("x1", delta, 0.786153);
	BayesNet<ConditionalGaussian>  actual1 = bayesTree.marginal<LinearFactor>("x1");
	CHECK(assert_equal((BayesNet<ConditionalGaussian>)expected1,actual1,1e-4));

	// Check marginal on x2
  GaussianBayesNet expected2("x2", delta, 0.687131);
	BayesNet<ConditionalGaussian>  actual2 = bayesTree.marginal<LinearFactor>("x2");
	CHECK(assert_equal((BayesNet<ConditionalGaussian>)expected2,actual2,1e-4));

	// Check marginal on x3
  GaussianBayesNet expected3("x3", delta, 0.671512);
	BayesNet<ConditionalGaussian>  actual3 = bayesTree.marginal<LinearFactor>("x3");
	CHECK(assert_equal((BayesNet<ConditionalGaussian>)expected3,actual3,1e-4));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_shortcuts )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	GaussianBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);
	boost::shared_ptr<VectorConfig> actualSolution = chordalBayesNet->optimize();

	// Create the Bayes tree
	Gaussian bayesTree(*chordalBayesNet);
	Gaussian::sharedClique R = bayesTree.root();

	// Check the conditional P(Root|Root)
	BayesNet<ConditionalGaussian> empty;
	Gaussian::sharedBayesNet actual1 = R->shortcut<LinearFactor>(R);
	CHECK(assert_equal(empty,*actual1,1e-4));

	// Check the conditional P(C2|Root)
	Gaussian::sharedClique C2 = bayesTree["x3"];
	Gaussian::sharedBayesNet actual2 = C2->shortcut<LinearFactor>(R);
	CHECK(assert_equal(empty,*actual2,1e-4));

	// Check the conditional P(C3|Root), which should be equal to P(x2|x4)
	ConditionalGaussian::shared_ptr p_x2_x4 = (*chordalBayesNet)["x2"];
	BayesNet<ConditionalGaussian> expected3; expected3.push_back(p_x2_x4);
	Gaussian::sharedClique C3 = bayesTree["x1"];
	Gaussian::sharedBayesNet actual3 = C3->shortcut<LinearFactor>(R);
	CHECK(assert_equal(expected3,*actual3,1e-4));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_clique_marginals )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	GaussianBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);
	boost::shared_ptr<VectorConfig> actualSolution = chordalBayesNet->optimize();

	// Create the Bayes tree
	Gaussian bayesTree(*chordalBayesNet);
	Gaussian::sharedClique R = bayesTree.root();

	// Check the conditional P(C3|Root), which should be equal to P(x2|x4)
	GaussianBayesNet expected3("x2",zero(2),0.687131);
  Vector sigma3 = repeat(2, 0.707107);
  Matrix A12 = (-0.5)*eye(2);
	ConditionalGaussian::shared_ptr cg3(new ConditionalGaussian("x1", zero(2), eye(2), "x2", A12, sigma3));
	expected3.push_front(cg3);
	Gaussian::sharedClique C3 = bayesTree["x1"];
	BayesNet<ConditionalGaussian> actual3 = C3->marginal<LinearFactor>(R);
	CHECK(assert_equal((BayesNet<ConditionalGaussian>)expected3,actual3,1e-4));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
