/*
 * testGaussianJunctionTree.cpp
 *
 * Created on: Jul 8, 2010
 * @author Kai Ni
 */

#include <iostream>
#include <gtsam/CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/inference/BayesTree-inl.h>

using namespace std;
using namespace gtsam;

static const varid_t x2=0, x1=1, x3=2, x4=3;

GaussianFactorGraph createChain() {

	typedef GaussianFactorGraph::sharedFactor Factor;
	SharedDiagonal model(Vector_(1, 0.5));
	Factor factor1(new GaussianFactor(x2, Matrix_(1,1,1.), x1, Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor2(new GaussianFactor(x2, Matrix_(1,1,1.), x3, Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor3(new GaussianFactor(x3, Matrix_(1,1,1.), x4, Matrix_(1,1,1.), Vector_(1,1.),  model));
	Factor factor4(new GaussianFactor(x4, Matrix_(1,1,1.), Vector_(1,1.),  model));

	GaussianFactorGraph fg;
	fg.push_back(factor1);
	fg.push_back(factor2);
	fg.push_back(factor3);
	fg.push_back(factor4);

	return fg;
}

/* ************************************************************************* */
/**
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 *
 * x2 x1 x3 x4  b
 *  1  1        1
 *  1     1     1
 *        1  1  1
 *           1  1
 *
 *  1  0  0  1
 */
TEST( GaussianJunctionTree, eliminate )
{
	GaussianFactorGraph fg = createChain();
	GaussianJunctionTree junctionTree(fg);
	BayesTree<GaussianConditional>::sharedClique rootClique = junctionTree.eliminate();

	typedef BayesTree<GaussianConditional>::sharedConditional sharedConditional;
	Matrix two = Matrix_(1,1,2.);
	Matrix one = Matrix_(1,1,1.);
	BayesTree<GaussianConditional> bayesTree_expected;
	bayesTree_expected.insert(sharedConditional(new GaussianConditional(x4, Vector_(1,2.), two, Vector_(1,1.))));
	bayesTree_expected.insert(sharedConditional(new GaussianConditional(x3, Vector_(1,2.), two, x4, two, Vector_(1,1.))));
	bayesTree_expected.insert(sharedConditional(new GaussianConditional(x1, Vector_(1,0.), one*(-1), x3, one, Vector_(1,1.))));
	bayesTree_expected.insert(sharedConditional(new GaussianConditional(x2, Vector_(1,2.), two, x1, one, x3, one, Vector_(1,1.))));
	CHECK(assert_equal(*bayesTree_expected.root(), *rootClique));
	CHECK(assert_equal(*(bayesTree_expected.root()->children().front()), *(rootClique->children().front())));

}

/* ************************************************************************* */
TEST( GaussianJunctionTree, optimizeMultiFrontal )
{
	GaussianFactorGraph fg = createChain();
	GaussianJunctionTree tree(fg);

	VectorConfig actual = tree.optimize();
	VectorConfig expected(vector<size_t>(4,1));
	expected[x1] = Vector_(1, 0.);
	expected[x2] = Vector_(1, 1.);
	expected[x3] = Vector_(1, 0.);
	expected[x4] = Vector_(1, 1.);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianJunctionTree, complexExample) {

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
