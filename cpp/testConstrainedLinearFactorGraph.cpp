/**
 * @file testConstrainedLinearFactorGraph.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "ConstrainedLinearFactorGraph.h"
#include "LinearFactorGraph.h"
#include "smallExample.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, elimination1 )
{
	// create unary factor
	double sigma = 0.1;
	Matrix Ax = eye(2) / sigma;
	Vector b1(2);
	b1(0) = 1.0;
	b1(1) = -1.0;
	LinearFactor::shared_ptr f1(new LinearFactor("x", Ax, b1 / sigma));

	// create binary constraint factor
	Matrix Ax1(2, 2);
	Ax1(0, 0) = 1.0; Ax1(0, 1) = 2.0;
	Ax1(1, 0) = 2.0; Ax1(1, 1) = 1.0;
	Matrix Ay1 = eye(2) * 10;
	Vector b2 = Vector_(2, 1.0, 2.0);
	LinearConstraint::shared_ptr f2(
			new LinearConstraint("x", Ax1, "y", Ay1, b2));

	// construct the graph
	ConstrainedLinearFactorGraph fg;
	fg.push_back(f1);
	fg.push_back_constraint(f2);

	// verify construction of the graph
	CHECK(fg.size() == 2);

	// eliminate x
	Ordering ord;
	ord.push_back("x");
	ChordalBayesNet::shared_ptr cbn = fg.eliminate(ord);

	//verify changes and output
	CHECK(fg.size() == 1);
	CHECK(cbn->size() == 1);
	ConstrainedConditionalGaussian expectedCCG1(b2, Ax1, "y", Ay1);
	CHECK(expectedCCG1.equals(*(cbn->get("x"))));
	Matrix Ap(2,2);
	Ap(0, 0) =  1.0; Ap(0, 1) = -2.0;
	Ap(1, 0) = -2.0; Ap(1, 1) =  1.0;
	Ap = 33.3333 * Ap;
	Vector bp = Vector_(2, 0.0, -10.0);
	LinearFactor expectedLF("y", Ap, bp);
	CHECK(expectedLF.equals(*(fg[0]), 1e-4));

	// eliminate y
	Ordering ord2;
	ord2.push_back("y");
	cbn = fg.eliminate(ord2);
	CHECK(fg.size() == 0);
	Matrix Ar(2,2);
	Ar(0, 0) = 74.5356; Ar(0, 1) = -59.6285;
	Ar(1, 0) = 0.0;     Ar(1, 1) = 44.7214;
	Vector br = Vector_(2, 8.9443, 4.4721);
	ConditionalGaussian expected2(br, Ar);
	CHECK(expected2.equals(*(cbn->get("y"))));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, optimize )
{
	// create unary factor
	double sigma = 0.1;
	Matrix Ax = eye(2) / sigma;
	Vector b1(2);
	b1(0) = 1.0;
	b1(1) = -1.0;
	LinearFactor::shared_ptr f1(new LinearFactor("x", Ax, b1 / sigma));

	// create binary constraint factor
	Matrix Ax1(2, 2);
	Ax1(0, 0) = 1.0; Ax1(0, 1) = 2.0;
	Ax1(1, 0) = 2.0; Ax1(1, 1) = 1.0;
	Matrix Ay1 = eye(2) * 10;
	Vector b2 = Vector_(2, 1.0, 2.0);
	LinearConstraint::shared_ptr f2(
			new LinearConstraint("x", Ax1, "y", Ay1, b2));

	// construct the graph
	ConstrainedLinearFactorGraph fg;
	fg.push_back(f1);
	fg.push_back_constraint(f2);

	// perform optimization
	Ordering ord;
	ord.push_back("y");
	ord.push_back("x");
	FGConfig actual = fg.optimize(ord);

	FGConfig expected;
	expected.insert("x", Vector_(2, 1.0, -1.0));
	expected.insert("y", Vector_(2, 0.2,  0.1));

	CHECK(expected.size() == actual.size());
	CHECK(assert_equal(expected["x"], actual["x"], 1e-4));
	CHECK(assert_equal(expected["y"], actual["y"], 1e-4));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, is_constrained )
{
	// very simple check
	ConstrainedLinearFactorGraph fg;
	CHECK(!fg.is_constrained("x"));

	// create simple graph
	Vector b = Vector_(2, 0.0, 0.0);
	LinearFactor::shared_ptr f1(new LinearFactor("x", eye(2), "y", eye(2), b));
	LinearFactor::shared_ptr f2(new LinearFactor("z", eye(2), "w", eye(2), b));
	LinearConstraint::shared_ptr f3(new LinearConstraint("y", eye(2), "z", eye(2), b));
	fg.push_back(f1);
	fg.push_back(f2);
	fg.push_back_constraint(f3);

	CHECK(fg.is_constrained("y"));
	CHECK(fg.is_constrained("z"));
	CHECK(!fg.is_constrained("x"));
	CHECK(!fg.is_constrained("w"));
}

/* ************************************************************************* */
//TEST( ConstrainedLinearFactorGraph, basic )
//{
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//
//	// expected equality factor
//	Vector v1(2); v1(0)=1.;v1(1)=2.;
//	LinearConstraint::shared_ptr f1(new LinearConstraint(v1, "x0"));
//
//	// expected normal linear factor
//	Matrix A21(2,2);
//	A21(0,0) = -10 ; A21(0,1) =   0;
//	A21(1,0) =   0 ; A21(1,1) = -10;
//
//	Matrix A22(2,2);
//	A22(0,0) = 10 ; A22(0,1) =  0;
//	A22(1,0) =  0 ; A22(1,1) = 10;
//
//	Vector b(2);
//	b(0) = 20 ; b(1) = 30;
//
//	LinearFactor::shared_ptr f2(new LinearFactor("x0", A21,  "x1", A22, b));
//
//	CHECK(f2->equals(*(fg[0])));
//	CHECK(f1->equals(*(fg.eq_at(0))));
//}

//TEST ( ConstrainedLinearFactorGraph, copy )
//{
//	LinearFactorGraph lfg = createLinearFactorGraph();
//	LinearFactor::shared_ptr f1 = lfg[0];
//	LinearFactor::shared_ptr f2 = lfg[1];
//	LinearFactor::shared_ptr f3 = lfg[2];
//	LinearFactor::shared_ptr f4 = lfg[3];
//
//	ConstrainedLinearFactorGraph actual(lfg);
//
//	ConstrainedLinearFactorGraph expected;
//	expected.push_back(f1);
//	expected.push_back(f2);
//	expected.push_back(f3);
//	expected.push_back(f4);
//
//	CHECK(actual.equals(expected));
//}
//
//TEST( ConstrainedLinearFactorGraph, equals )
//{
//	// basic equality test
//	ConstrainedLinearFactorGraph fg  = createConstrainedLinearFactorGraph();
//	ConstrainedLinearFactorGraph fg2 = createConstrainedLinearFactorGraph();
//	CHECK( fg.equals(fg2) );
//
//	// ensuring that equality factors are compared
//	LinearFactor::shared_ptr f2 = fg[0]; // get a linear factor from existing graph
//	ConstrainedLinearFactorGraph fg3;
//	fg3.push_back(f2);
//	CHECK( !fg3.equals(fg) );
//}
//
//TEST( ConstrainedLinearFactorGraph, size )
//{
//	LinearFactorGraph lfg = createLinearFactorGraph();
//	ConstrainedLinearFactorGraph fg1(lfg);
//
//	CHECK(fg1.size() == lfg.size());
//
//	ConstrainedLinearFactorGraph fg2 = createConstrainedLinearFactorGraph();
//
//	CHECK(fg2.size() == 2);
//}
//
//TEST( ConstrainedLinearFactorGraph, is_constrained )
//{
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//
//	CHECK(fg.is_constrained("x0"));
//	CHECK(!fg.is_constrained("x1"));
//}
//
//TEST( ConstrainedLinearFactorGraph, optimize )
//{
//	ConstrainedLinearFactorGraph fg1 = createConstrainedLinearFactorGraph();
//	ConstrainedLinearFactorGraph fg2 = createConstrainedLinearFactorGraph();
//
//	FGConfig expected = createConstrainedConfig();
//
//	Ordering ord1;
//	ord1.push_back("x0");
//	ord1.push_back("x1");
//
//	Ordering ord2;
//	ord2.push_back("x1");
//	ord2.push_back("x0");
//
//	FGConfig actual1 = fg1.optimize(ord1);
//	FGConfig actual2 = fg2.optimize(ord2);
//
//	CHECK(actual1.equals(expected));
//	CHECK(actual1.equals(actual2));
//}
//
//TEST (ConstrainedLinearFactorGraph, eliminate )
//{
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//	FGConfig c = createConstrainedConfig();
//
//	Ordering ord1;
//	ord1.push_back("x0");
//	ord1.push_back("x1");
//
//	ConstrainedChordalBayesNet::shared_ptr actual = fg.eliminate(ord1);
//
//	// create an expected bayes net
//	ConstrainedChordalBayesNet::shared_ptr expected(new ConstrainedChordalBayesNet);
//
//	ConstrainedConditionalGaussian::shared_ptr d(new ConstrainedConditionalGaussian);//(c["x0"], "x0"));
//	expected->insert_df("x0", d);
//
//	Matrix A = eye(2);
//	double sigma = 0.1;
//	Vector dv = c["x1"];
//	ConditionalGaussian::shared_ptr cg(new ConditionalGaussian(dv/sigma, A/sigma));
//	expected->insert("x1", cg);
//
//	CHECK(actual->equals(*expected));
//}
//
//TEST (ConstrainedLinearFactorGraph, baseline_optimize)
//{
//	// tests performance when there are no equality factors in the graph
//	LinearFactorGraph lfg = createLinearFactorGraph();
//	ConstrainedLinearFactorGraph clfg(lfg); // copy in the linear factor graph
//
//	Ordering ord;
//	ord.push_back("l1");
//	ord.push_back("x1");
//	ord.push_back("x2");
//
//	FGConfig actual = clfg.optimize(ord);
//
//	FGConfig expected = lfg.optimize(ord); // should be identical to regular lfg optimize
//
//	CHECK(actual.equals(expected));
//}
//
//TEST (ConstrainedLinearFactorGraph, baseline_eliminate_one )
//{
//	  LinearFactorGraph fg = createLinearFactorGraph();
//	  ConstrainedLinearFactorGraph cfg(fg);
//
//	  ConditionalGaussian::shared_ptr actual = cfg.eliminate_one("x1");
//
//	  // create expected Conditional Gaussian
//	  Matrix R11 = Matrix_(2,2,
//				 15.0, 00.0,
//				 00.0, 15.0
//				 );
//	  Matrix S12 = Matrix_(2,2,
//				 -1.66667, 0.00,
//				 +0.00,-1.66667
//				 );
//	  Matrix S13 = Matrix_(2,2,
//				 -6.66667, 0.00,
//				 +0.00,-6.66667
//				 );
//	  Vector d(2); d(0) = -2; d(1) = -1.0/3.0;
//	  ConditionalGaussian expected(d,R11,"l1",S12,"x2",S13);
//
//	  CHECK( actual->equals(expected) );
//}
//
//TEST (ConstrainedLinearFactorGraph, eliminate_constraint)
//{
////	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
////	ConstrainedConditionalGaussian::shared_ptr actual = fg.eliminate_constraint("x0");
////
////	FGConfig c = createConstrainedConfig();
////	ConstrainedConditionalGaussian::shared_ptr expected(new ConstrainedConditionalGaussian);//(c["x0"], "x0"));
////
////	CHECK(assert_equal(*actual, *expected)); // check output for correct delta function
////
////	CHECK(fg.size() == 1); // check size
////
////	ConstrainedLinearFactorGraph::eq_const_iterator eit = fg.eq_begin();
////	CHECK(eit == fg.eq_end()); // ensure no remaining equality factors
////
////	// verify the remaining factor - should be a unary factor on x1
////	ConstrainedLinearFactorGraph::const_iterator it = fg.begin();
////	LinearFactor::shared_ptr factor_actual = *it;
////
////	CHECK(factor_actual->size() == 1);
//}
//
//TEST (ConstrainedLinearFactorGraph, constraintCombineAndEliminate )
//{
//	// create a set of factors
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//	LinearConstraint::shared_ptr eq = fg.eq_at(0);
//	LinearFactor::shared_ptr f1 = fg[0];
//
//	// make a joint linear factor
//	set<LinearFactor::shared_ptr> f1_set;
//	f1_set.insert(f1);
//	boost::shared_ptr<MutableLinearFactor> joined(new MutableLinearFactor(f1_set));
//
//	// create a sample graph
//	ConstrainedLinearFactorGraph graph;
//
//	// combine linear factor and eliminate
//	graph.constraintCombineAndEliminate(*eq, *joined);
//
//	// verify structure
//	CHECK(graph.size() == 1); // will have only one factor
//	LinearFactor::shared_ptr actual = graph[0];
//	CHECK(actual->size() == 1); // remaining factor will be unary
//
//	// verify values
//	FGConfig c = createConstrainedConfig();
//	Vector exp_v = c["x1"];
//	Matrix A = actual->get_A("x1");
//	Vector b = actual->get_b();
//	Vector act_v = backsubstitution(A, b);
//	CHECK(assert_equal(act_v, exp_v));
//}
//
//TEST (ConstrainedLinearFactorGraph, extract_eq)
//{
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//	LinearConstraint::shared_ptr actual = fg.extract_eq("x0");
//
//	Vector v1(2); v1(0)=1.;v1(1)=2.;
//	LinearConstraint::shared_ptr expected(new LinearConstraint(v1, "x0"));
//
//	// verify output
//	CHECK(assert_equal(*actual, *expected));
//
//	// verify removal
//	ConstrainedLinearFactorGraph::eq_const_iterator it = fg.eq_begin();
//	CHECK(it == fg.eq_end());
//
//	// verify full size
//	CHECK(fg.size() == 1);
//}
//
//TEST( ConstrainedLinearFactorGraph, GET_ORDERING)
//{
//  ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//  Ordering ord = fg.getOrdering();
//  CHECK(ord[0] == string("x0"));
//  CHECK(ord[1] == string("x1"));
//}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

