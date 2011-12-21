/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic IndexFactor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

using namespace std;
using namespace gtsam;

static const Index vx2 = 0;
static const Index vx1 = 1;
static const Index vl1 = 2;

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, EliminateOne )
//{
//	// create a test graph
//	SymbolicFactorGraph fg;
//	fg.push_factor(vx2, vx1);
//
//	SymbolicSequentialSolver::EliminateUntil(fg, vx2+1);
//	SymbolicFactorGraph expected;
//	expected.push_back(boost::shared_ptr<IndexFactor>());
//	expected.push_factor(vx1);
//
//	CHECK(assert_equal(expected, fg));
//}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, SymbolicSequentialSolver )
{
	// create factor graph
	SymbolicFactorGraph g;
	g.push_factor(vx2, vx1, vl1);
	g.push_factor(vx1, vl1);
	g.push_factor(vx1);
	// test solver is Testable
	SymbolicSequentialSolver solver(g);
//	GTSAM_PRINT(solver);
	EXPECT(assert_equal(solver,solver));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, constructFromBayesNet )
{
	// create expected factor graph
	SymbolicFactorGraph expected;
	expected.push_factor(vx2, vx1, vl1);
	expected.push_factor(vx1, vl1);
	expected.push_factor(vx1);

	// create Bayes Net
	IndexConditional::shared_ptr x2(new IndexConditional(vx2, vx1, vl1));
	IndexConditional::shared_ptr l1(new IndexConditional(vx1, vl1));
	IndexConditional::shared_ptr x1(new IndexConditional(vx1));

	BayesNet<IndexConditional> bayesNet;
	bayesNet.push_back(x2);
	bayesNet.push_back(l1);
	bayesNet.push_back(x1);

	// create actual factor graph from a Bayes Net
	SymbolicFactorGraph actual(bayesNet);

	CHECK(assert_equal((SymbolicFactorGraph)expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
	// Create two factor graphs and expected combined graph
	SymbolicFactorGraph fg1, fg2, expected;

	fg1.push_factor(vx1);
	fg1.push_factor(vx2, vx1);

	fg2.push_factor(vx1, vl1);
	fg2.push_factor(vx2, vl1);

	expected.push_factor(vx1);
	expected.push_factor(vx2, vx1);
	expected.push_factor(vx1, vl1);
	expected.push_factor(vx2, vl1);

	// combine
	SymbolicFactorGraph actual = combine(fg1, fg2);
	CHECK(assert_equal(expected, actual));

	// combine using push_back
	fg1.push_back(fg2);
	CHECK(assert_equal(expected, fg1));
}

/* ************************************************************************* */

///**
// * An elimination tree is a tree of factors
// */
//class ETree: public Testable<ETree> {
//
//public:
//
//	typedef boost::shared_ptr<IndexFactor> sharedFactor;
//	typedef boost::shared_ptr<ETree> shared_ptr;
//
//private:
//
//	Index key_; /** index associated with root */
//	list<sharedFactor> factors_; /** factors associated with root */
//	list<shared_ptr> subTrees_; /** sub-trees */
//
//	typedef pair<SymbolicBayesNet, IndexFactor> Result;
//
//	/**
//	 * Recursive routine that eliminates the factors arranged in an elimination tree
//	 */
//	Result eliminate_() const {
//
//	  SymbolicBayesNet bn;
//
//	  set<Index> separator;
//
//	  // loop over all factors associated with root
//	  // and set-union their keys to a separator
//	  BOOST_FOREACH(const sharedFactor& factor, factors_)
//	  BOOST_FOREACH(Index key, *factor) {
//	    if (key != key_) separator.insert(key); }
//
//	  // for all children, eliminate into Bayes net
//	  BOOST_FOREACH(const shared_ptr& child, subTrees_) {
//	    Result result = child->eliminate_();
//	    bn.push_back(result.first);
//	    BOOST_FOREACH(Index key, result.second)
//	    if (key != key_) separator.insert(key);
//	  }
//
//	  // Make the conditional from the key and separator, and insert it in Bayes net
//	  vector<Index> parents;
//	  std::copy(separator.begin(), separator.end(), back_inserter(parents));
//	  IndexConditional::shared_ptr conditional(new IndexConditional(key_, parents));
//	  bn.push_back(conditional);
//
//	  // now create the new factor from separator to return to caller
//	  IndexFactor newFactor(separator.begin(), separator.end());
//	  return Result(bn, newFactor);
//	}
//
//public:
//
//	/** default constructor */
//	ETree(Index key = 0) :
//		key_(key) {
//	}
//
//	/** add a factor */
//	void add(const sharedFactor& factor) {
//		factors_.push_back(factor);
//	}
//
//	/** add a subtree */
//	void add(const shared_ptr& child) {
//		subTrees_.push_back(child);
//	}
//
//	void print(const std::string& name) const {
//		cout << name << " (" << key_ << ")" << endl;
//		BOOST_FOREACH(const sharedFactor& factor, factors_)
//						factor->print(name + "  ");
//		BOOST_FOREACH(const shared_ptr& child, subTrees_)
//						child->print(name + "  ");
//	}
//
//	bool equals(const ETree& expected, double tol) const {
//		// todo
//		return false;
//	}
//
//	/**
//	 * Eliminate the factors to a Bayes Net
//	 */
//	SymbolicBayesNet eliminate() const {
//
//		// call recursive routine
//		Result result = eliminate_();
//		return result.first;
//	}
//
//};
//
//// build hardcoded tree
//ETree::shared_ptr buildHardcodedTree(const SymbolicFactorGraph& fg) {
//
//	ETree::shared_ptr leaf0(new ETree);
//	leaf0->add(fg[0]);
//	leaf0->add(fg[1]);
//
//	ETree::shared_ptr node1(new ETree(1));
//	node1->add(fg[2]);
//	node1->add(leaf0);
//
//	ETree::shared_ptr node2(new ETree(2));
//	node2->add(fg[3]);
//	node2->add(node1);
//
//	ETree::shared_ptr leaf3(new ETree(3));
//	leaf3->add(fg[4]);
//
//	ETree::shared_ptr etree(new ETree(4));
//	etree->add(leaf3);
//	etree->add(node2);
//
//	return etree;
//}
//
//typedef size_t RowIndex;
//typedef vector<list<RowIndex> > StructA;
//typedef vector<boost::optional<Index> > optionalIndices;
//
///**
// * Gilbert01bit algorithm in Figure 2.2
// */
//optionalIndices buildETree(const StructA& structA) {
//
//  // todo: get n
//  size_t n = 5;
//  optionalIndices parent(n);
//
//  // todo: get m
//  size_t m = 5;
//  optionalIndices prev_col(m);
//
//  // for column j \in 1 to n do
//  for (Index j = 0; j < n; j++) {
//    // for row i \in Struct[A*j] do
//    BOOST_FOREACH(RowIndex i, structA[j]) {
//      if (prev_col[i]) {
//        Index k = *(prev_col[i]);
//        // find root r of the current tree that contains k
//        Index r = k;
//        while (parent[r])
//          r = *parent[r];
//        if (r != j) parent[r].reset(j);
//      }
//      prev_col[i].reset(j);
//    }
//  }
//
//  return parent;
//}
//
///**
// * TODO: Build StructA from factor graph
// */
//StructA createStructA(const SymbolicFactorGraph& fg) {
//
//	StructA structA;
//
//	// hardcode for now
//	list<RowIndex> list0;
//	list0 += 0, 1;
//	structA.push_back(list0);
//	list<RowIndex> list1;
//	list1 += 0, 2;
//	structA.push_back(list1);
//	list<RowIndex> list2;
//	list2 += 1, 3;
//	structA.push_back(list2);
//	list<RowIndex> list3;
//	list3 += 4;
//	structA.push_back(list3);
//	list<RowIndex> list4;
//	list4 += 2, 3, 4;
//	structA.push_back(list4);
//
//	return structA;
//}
//
///**
// * Build ETree from factor graph and parent indices
// */
//ETree::shared_ptr buildETree(const SymbolicFactorGraph& fg, const optionalIndices& parent) {
//
//	// todo: get n
//	size_t n = 5;
//
//	// Create tree structure
//	vector<ETree::shared_ptr> trees(n);
//	for (Index k = 1; k <= n; k++) {
//		Index j = n - k;
//		trees[j].reset(new ETree(j));
//		if (parent[j]) trees[*parent[j]]->add(trees[j]);
//	}
//
//	// Hang factors in right places
//	BOOST_FOREACH(const ETree::sharedFactor& factor, fg)
//				{
//					Index j = factor->front();
//					trees[j]->add(factor);
//				}
//
//	return trees[n - 1];
//}
//
///* ************************************************************************* */
//// Test of elimination tree creation
//// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
///* ************************************************************************* */
///**
// * Build ETree from factor graph
// */
//ETree::shared_ptr buildETree(const SymbolicFactorGraph& fg) {
//
//	// create vector of factor indices
//	StructA structA = createStructA(fg);
//
//	// call Gilbert01bit algorithm
//	optionalIndices parent = buildETree(structA);
//
//	// Build ETree from factor graph and parent indices
//	return buildETree(fg,  parent);
//}
//
//TEST( ETree, buildETree )
//{
//	// create example factor graph
//	SymbolicFactorGraph fg;
//	fg.push_factor(0, 1);
//	fg.push_factor(0, 2);
//	fg.push_factor(1, 4);
//	fg.push_factor(2, 4);
//	fg.push_factor(3, 4);
//
//	ETree::shared_ptr expected = buildHardcodedTree(fg);
//
//	// Build from factor graph
//	ETree::shared_ptr actual = buildETree(fg);
//
//	// todo: CHECK(assert_equal(*expected,*actual));
//}
//
///* ************************************************************************* */
//// Test to drive elimination tree development
//// graph: f(0,1) f(0,2) f(1,4) f(2,4) f(3,4)
///* ************************************************************************* */
//
///**
// * Eliminate factor graph
// */
//SymbolicBayesNet eliminate(const SymbolicFactorGraph& fg) {
//
//	// build etree
//	ETree::shared_ptr etree = buildETree(fg);
//
//	return etree->eliminate();
//}

//TEST( SymbolicFactorGraph, eliminate )
//{
//	// create expected Chordal bayes Net
//	IndexConditional::shared_ptr c0(new IndexConditional(0, 1, 2));
//	IndexConditional::shared_ptr c1(new IndexConditional(1, 2, 4));
//	IndexConditional::shared_ptr c2(new IndexConditional(2, 4));
//	IndexConditional::shared_ptr c3(new IndexConditional(3, 4));
//	IndexConditional::shared_ptr c4(new IndexConditional(4));
//
//	SymbolicBayesNet expected;
//	expected.push_back(c3);
//	expected.push_back(c0);
//	expected.push_back(c1);
//	expected.push_back(c2);
//	expected.push_back(c4);
//
//	// Create factor graph
//	SymbolicFactorGraph fg;
//	fg.push_factor(0, 1);
//	fg.push_factor(0, 2);
//	fg.push_factor(1, 4);
//	fg.push_factor(2, 4);
//	fg.push_factor(3, 4);
//
//	// eliminate
//	SymbolicBayesNet actual = eliminate(fg);
//
//	CHECK(assert_equal(expected,actual));
//}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
