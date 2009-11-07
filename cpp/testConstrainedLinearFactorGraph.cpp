/**
 * @file testConstrainedLinearFactorGraph.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "ConstrainedLinearFactorGraph.h"
#include "LinearFactorGraph.h"
#include "Ordering.h"
#include "smallExample.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, elimination1 )
{
	// get the graph
	// *-X-x-Y
	ConstrainedLinearFactorGraph fg = createSingleConstraintGraph();

	// verify construction of the graph
	CHECK(fg.size() == 2);

	// eliminate x
	Ordering ord;
	ord.push_back("x");
	GaussianBayesNet::shared_ptr cbn = fg.eliminate(ord);

	// verify result of elimination
	// CBN of size 1, as we only eliminated X now
	CHECK(fg.size() == 1);
	CHECK(cbn->size() == 1);

	// We will have a "delta function" on X as a function of Y
	// |1 2||x_1| = |1| - |10 0||y_1|
	// |2 1||x_2|   |2|   |0 10||y_2|
	Matrix Ax1(2, 2);
	Ax1(0, 0) = 1.0; Ax1(0, 1) = 2.0;
	Ax1(1, 0) = 2.0; Ax1(1, 1) = 1.0;
	Matrix Ay1 = eye(2) * 10;
	Vector b2 = Vector_(2, 1.0, 2.0);
	ConstrainedConditionalGaussian expectedCCG1("x",b2, Ax1, "y", Ay1);
	CHECK(expectedCCG1.equals(*((*cbn)["x"])));

	// verify remaining factor on y
	// Gaussian factor on X becomes different Gaussian factor on Y
	Matrix Ap(2,2);
	Ap(0, 0) =  1.0; Ap(0, 1) = -2.0;
	Ap(1, 0) = -2.0; Ap(1, 1) =  1.0;
	Ap = 33.3333 * Ap;
	Vector bp = Vector_(2, 0.0, -10.0);
	double sigma1 = 1;
	LinearFactor expectedLF("y", Ap, bp,sigma1);
	CHECK(expectedLF.equals(*(fg[0]), 1e-4));

	// eliminate y
	Ordering ord2;
	ord2.push_back("y");
	cbn = fg.eliminate(ord2);

	// Check result
	CHECK(fg.size() == 0);
	Matrix R(2,2);
	R(0, 0) = 74.5356; R(0, 1) = -59.6285;
	R(1, 0) = 0.0;     R(1, 1) = 44.7214;
	Vector br = Vector_(2, 8.9443, 4.4721);
	Vector tau(2);
	tau(0) = R(0,0);
	tau(1) = R(1,1);

	// normalize the existing matrices
	Matrix N = eye(2,2);
	N(0,0) = 1/tau(0);
	N(1,1) = 1/tau(1);
	R = N*R;
	ConditionalGaussian expected2("y",br, R, tau);
	CHECK(expected2.equals(*((*cbn)["y"])));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, optimize )
{
	// create graph
	ConstrainedLinearFactorGraph fg = createSingleConstraintGraph();

	// perform optimization
	Ordering ord;
	ord.push_back("y");
	ord.push_back("x");
	VectorConfig actual = fg.optimize(ord);

	VectorConfig expected;
	expected.insert("x", Vector_(2, 1.0, -1.0));
	expected.insert("y", Vector_(2, 0.2,  0.1));

	CHECK(expected.size() == actual.size());
	CHECK(assert_equal(expected["x"], actual["x"], 1e-4));
	CHECK(assert_equal(expected["y"], actual["y"], 1e-4));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, optimize2 )
{
	// create graph
	ConstrainedLinearFactorGraph fg = createSingleConstraintGraph();

	// perform optimization
	Ordering ord;
	ord.push_back("x");
	ord.push_back("y");
	VectorConfig actual = fg.optimize(ord);

	VectorConfig expected;
	expected.insert("x", Vector_(2, 1.0, -1.0));
	expected.insert("y", Vector_(2, 0.2,  0.1));

	CHECK(expected.size() == actual.size());
	CHECK(assert_equal(expected["x"], actual["x"], 1e-4)); // Fails here: gets x = (-3, 1)
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
	LinearFactor::shared_ptr f1(new LinearFactor("x", eye(2), "y", eye(2), b,1));
	LinearFactor::shared_ptr f2(new LinearFactor("z", eye(2), "w", eye(2), b,1));
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
TEST( ConstrainedLinearFactorGraph, get_constraint_separator )
{
	ConstrainedLinearFactorGraph fg1 = createMultiConstraintGraph();
	ConstrainedLinearFactorGraph fg2 = createMultiConstraintGraph();
	LinearConstraint::shared_ptr lc1 = fg1.constraint_at(0);
	LinearConstraint::shared_ptr lc2 = fg1.constraint_at(1);

	vector<LinearConstraint::shared_ptr> actual1 = fg1.find_constraints_and_remove("y");
	CHECK(fg1.size() == 2);
	CHECK(actual1.size() == 1);
	CHECK((*actual1.begin())->equals(*lc1));

	vector<LinearConstraint::shared_ptr> actual2 = fg2.find_constraints_and_remove("x");
	CHECK(fg2.size() == 1);
	CHECK(actual2.size() == 2);
	CHECK((*actual1.begin())->equals(*lc1));
	LinearConstraint::shared_ptr act = *(++actual2.begin());
	CHECK(act->equals(*lc2));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, update_constraints )
{
	// create a graph
	ConstrainedLinearFactorGraph fg1 = createMultiConstraintGraph();

	// process constraints - picking first constraint on x
	vector<LinearConstraint::shared_ptr> constraints = fg1.find_constraints_and_remove("x");
	CHECK(constraints.size() == 2);
	CHECK(fg1.size() == 1); // both constraints removed
	LinearConstraint::shared_ptr primary = constraints[0];
	LinearConstraint::shared_ptr updatee = constraints[1];
	fg1.update_constraints("x", constraints, primary);
	CHECK(fg1.size() == 2); // induced constraint added back

	// expected induced constraint
	Matrix Ar(2,2);
	Ar(0, 0) = -16.6666; Ar(0, 1) = -6.6666;
	Ar(1, 0) = 10.0;     Ar(1, 1) = 0.0;
	Matrix A22(2,2);
	A22(0,0) = 1.0 ; A22(0,1) = 1.0;
	A22(1,0) = 1.0 ; A22(1,1) = 2.0;
	Vector br = Vector_(2, 0.0, 5.0);
	LinearConstraint::shared_ptr exp(new LinearConstraint("y", Ar, "z", A22, br));

	// evaluate
	CHECK(assert_equal(*(fg1.constraint_at(0)), *exp, 1e-4));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, find_constraints_and_remove )
{
	// constraint 1
	Matrix A11(2,2);
	A11(0,0) = 1.0 ; A11(0,1) = 2.0;
	A11(1,0) = 2.0 ; A11(1,1) = 1.0;

	Matrix A12(2,2);
	A12(0,0) = 10.0 ; A12(0,1) = 0.0;
	A12(1,0) = 0.0 ; A12(1,1) = 10.0;

	Vector b1(2);
	b1(0) = 1.0; b1(1) = 2.0;
	LinearConstraint::shared_ptr lc1(new LinearConstraint("x", A11, "y", A12, b1));

	// constraint 2
	Matrix A21(2,2);
	A21(0,0) =  3.0 ; A21(0,1) =  4.0;
	A21(1,0) = -1.0 ; A21(1,1) = -2.0;

	Matrix A22(2,2);
	A22(0,0) = 1.0 ; A22(0,1) = 1.0;
	A22(1,0) = 1.0 ; A22(1,1) = 2.0;

	Vector b2(2);
	b2(0) = 3.0; b2(1) = 4.0;
	LinearConstraint::shared_ptr lc2(new LinearConstraint("x", A21, "z", A22, b2));

	// construct the graph
	ConstrainedLinearFactorGraph fg1;
	fg1.push_back_constraint(lc1);
	fg1.push_back_constraint(lc2);

	// constraints on x
	vector<LinearConstraint::shared_ptr> expected1, actual1;
	expected1.push_back(lc1);
	expected1.push_back(lc2);
	actual1 = fg1.find_constraints_and_remove("x");
	CHECK(fg1.size() == 0);
	CHECK(expected1.size() == actual1.size());
	vector<LinearConstraint::shared_ptr>::const_iterator exp1, act1;
	for(exp1=expected1.begin(), act1=actual1.begin(); act1 != actual1.end(); ++act1, ++exp1) {
		CHECK((*exp1)->equals(**act1));
	}
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, eliminate_multi_constraint )
{
	ConstrainedLinearFactorGraph fg = createMultiConstraintGraph();

	// eliminate the constraint
	ConstrainedConditionalGaussian::shared_ptr cg1 = fg.eliminate_constraint("x");
	CHECK(cg1->nrParents() == 1);
	CHECK(fg.nrFactors() == 1);

	// eliminate the induced constraint
	ConstrainedConditionalGaussian::shared_ptr cg2 = fg.eliminate_constraint("y");
	CHECK(cg2->nrParents() == 1);
	CHECK(fg.nrFactors() == 0);

	// eliminate the linear factor
	ConditionalGaussian::shared_ptr cg3 = fg.eliminateOne("z");
	CHECK(cg3->nrParents() == 0);
	CHECK(fg.size() == 0);

	// solve piecewise
	VectorConfig actual;
	Vector act_z = cg3->solve(actual);
	actual.insert("z", act_z);
	CHECK(assert_equal(act_z, Vector_(2, -4.0, 5.0), 1e-4));
	Vector act_y = cg2->solve(actual);
	actual.insert("y", act_y);
	CHECK(assert_equal(act_y, Vector_(2, -0.1, 0.4), 1e-4));
	Vector act_x = cg1->solve(actual);
	CHECK(assert_equal(act_x, Vector_(2, -2.0, 2.0), 1e-4));
}

/* ************************************************************************* */
TEST( ConstrainedLinearFactorGraph, optimize_multi_constraint )
{
	ConstrainedLinearFactorGraph fg = createMultiConstraintGraph();
	// solve the graph
	Ordering ord;
	ord.push_back("x");
	ord.push_back("y");
	ord.push_back("z");

	VectorConfig actual = fg.optimize(ord);

	// verify
	VectorConfig expected;
	expected.insert("x", Vector_(2, -2.0, 2.0));
	expected.insert("y", Vector_(2, -0.1, 0.4));
	expected.insert("z", Vector_(2, -4.0, 5.0));
	CHECK(expected.size() == actual.size());
	CHECK(assert_equal(expected["x"], actual["x"], 1e-4));
	CHECK(assert_equal(expected["y"], actual["y"], 1e-4));
	CHECK(assert_equal(expected["z"], actual["z"], 1e-4));
}

/* ************************************************************************* */
//  OLD TESTS - should be ported into the new structure when possible
/* ************************************************************************* */

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
//	VectorConfig expected = createConstrainedConfig();
//
//	Ordering ord1;
//	ord1.push_back("x0");
//	ord1.push_back("x1");
//
//	Ordering ord2;
//	ord2.push_back("x1");
//	ord2.push_back("x0");
//
//	VectorConfig actual1 = fg1.optimize(ord1);
//	VectorConfig actual2 = fg2.optimize(ord2);
//
//	CHECK(actual1.equals(expected));
//	CHECK(actual1.equals(actual2));
//}
//
//TEST (ConstrainedLinearFactorGraph, eliminate )
//{
//	ConstrainedLinearFactorGraph fg = createConstrainedLinearFactorGraph();
//	VectorConfig c = createConstrainedConfig();
//
//	Ordering ord1;
//	ord1.push_back("x0");
//	ord1.push_back("x1");
//
//	ConstrainedGaussianBayesNet::shared_ptr actual = fg.eliminate(ord1);
//
//	// create an expected bayes net
//	ConstrainedGaussianBayesNet::shared_ptr expected(new ConstrainedGaussianBayesNet);
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
//	VectorConfig actual = clfg.optimize(ord);
//
//	VectorConfig expected = lfg.optimize(ord); // should be identical to regular lfg optimize
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
////	VectorConfig c = createConstrainedConfig();
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
//	boost::shared_ptr<LinearFactor> joined(new LinearFactor(f1_set));
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
//	VectorConfig c = createConstrainedConfig();
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

