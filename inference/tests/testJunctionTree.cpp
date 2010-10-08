/**
 * @file    testJunctionTree.cpp
 * @brief   Unit tests for Junction Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/inference/ClusterTree-inl.h>
#include <gtsam/inference/JunctionTree-inl.h>
#include <gtsam/inference/Factor-inl.h>

using namespace gtsam;

typedef JunctionTree<SymbolicFactorGraph> SymbolicJunctionTree;
typedef BayesTree<Conditional> SymbolicBayesTree;

/* ************************************************************************* *
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 ****************************************************************************/
TEST( JunctionTree, constructor )
{
  const varid_t x2=0, x1=1, x3=2, x4=3;
	SymbolicFactorGraph fg;
	fg.push_factor(x2,x1);
	fg.push_factor(x2,x3);
	fg.push_factor(x3,x4);

	SymbolicJunctionTree actual(fg);

	vector<varid_t> frontal1; frontal1 += x3, x4;
	vector<varid_t> frontal2; frontal2 += x2, x1;
	vector<varid_t> sep1;
	vector<varid_t> sep2; sep2 += x3;
	CHECK(assert_equal(frontal1, actual.root()->frontal));
	CHECK(assert_equal(sep1,     actual.root()->separator));
	LONGS_EQUAL(1,               actual.root()->size());
	CHECK(assert_equal(frontal2, actual.root()->children().front()->frontal));
	CHECK(assert_equal(sep2,     actual.root()->children().front()->separator));
	LONGS_EQUAL(2,               actual.root()->children().front()->size());
	CHECK(assert_equal(*fg[2], *(*actual.root())[0]));
  CHECK(assert_equal(*fg[0], *(*actual.root()->children().front())[0]));
  CHECK(assert_equal(*fg[1], *(*actual.root()->children().front())[1]));
}

/* ************************************************************************* *
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 ****************************************************************************/
TEST( JunctionTree, eliminate)
{
  const varid_t x2=0, x1=1, x3=2, x4=3;
  SymbolicFactorGraph fg;
  fg.push_factor(x2,x1);
  fg.push_factor(x2,x3);
  fg.push_factor(x3,x4);

  SymbolicJunctionTree jt(fg);
  SymbolicBayesTree::sharedClique actual = jt.eliminate();

  BayesNet<Conditional> bn = *Inference::Eliminate(fg);
  SymbolicBayesTree expected(bn);

//  cout << "BT from JT:\n";
//  actual->printTree("");

  CHECK(assert_equal(*expected.root(), *actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
