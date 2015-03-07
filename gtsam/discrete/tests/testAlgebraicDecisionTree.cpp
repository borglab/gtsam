/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file		testDecisionTree.cpp
 * @brief		Develop DecisionTree
 * @author	Frank Dellaert
 * @date	Mar 6, 2011
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteKey.h> // make sure we have traits
// headers first to make sure no missing headers
//#define DT_NO_PRUNING
#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DecisionTree-inl.h> // for convert only
#define DISABLE_TIMING

#include <boost/timer.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/assign/std/map.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/Signature.h>

using namespace std;
using namespace gtsam;

/* ******************************************************************************** */
typedef AlgebraicDecisionTree<Index> ADT;

template class DecisionTree<Index,double>;
template class AlgebraicDecisionTree<Index>;

#define DISABLE_DOT

template<typename T>
void dot(const T&f, const string& filename) {
#ifndef DISABLE_DOT
	f.dot(filename);
#endif
}

/** I can't get this to work !
 class Mul: boost::function<double(const double&, const double&)> {
 inline double operator()(const double& a, const double& b) {
 return a * b;
 }
 };

 // If second argument of binary op is Leaf
 template<typename L>
 typename DecisionTree<L, double>::Node::Ptr DecisionTree<L, double>::Choice::apply_fC_op_gL(
 Cache& cache, const Leaf& gL, Mul op) const {
 Ptr h(new Choice(label(), cardinality()));
 BOOST_FOREACH(const NodePtr& branch, branches_)
 h->push_back(branch->apply_f_op_g(cache, gL, op));
 return Unique(cache, h);
 }
 */

/* ******************************************************************************** */
// instrumented operators
/* ******************************************************************************** */
size_t muls = 0, adds = 0;
boost::timer timer;
void resetCounts() {
	muls = 0;
	adds = 0;
	timer.restart();
}
void printCounts(const string& s) {
#ifndef DISABLE_TIMING
	cout << boost::format("%s: %3d muls, %3d adds, %g ms.") % s % muls % adds
	% (1000 * timer.elapsed()) << endl;
#endif
	resetCounts();
}
double mul(const double& a, const double& b) {
	muls++;
	return a * b;
}
double add_(const double& a, const double& b) {
	adds++;
	return a + b;
}

/* ******************************************************************************** */
// test ADT
TEST(ADT, example3)
{
	// Create labels
	DiscreteKey A(0,2), B(1,2), C(2,2), D(3,2), E(4,2);

	// Literals
	ADT a(A, 0.5, 0.5);
	ADT notb(B, 1, 0);
	ADT c(C, 0.1, 0.9);
	ADT d(D, 0.1, 0.9);
	ADT note(E, 0.9, 0.1);

	ADT cnotb = c * notb;
	dot(cnotb, "ADT-cnotb");

//  a.print("a: ");
//  cnotb.print("cnotb: ");
	ADT acnotb = a * cnotb;
//	acnotb.print("acnotb: ");
//	acnotb.printCache("acnotb Cache:");

	dot(acnotb, "ADT-acnotb");


	ADT big = apply(apply(d, note, &mul), acnotb, &add_);
	dot(big, "ADT-big");
}

/* ******************************************************************************** */
// Asia Bayes Network
/* ******************************************************************************** */

/** Convert Signature into CPT */
ADT create(const Signature& signature) {
	ADT p(signature.discreteKeysParentsFirst(), signature.cpt());
	static size_t count = 0;
	const DiscreteKey& key = signature.key();
	string dotfile = (boost::format("CPT-%03d-%d") % ++count % key.first).str();
	dot(p, dotfile);
	return p;
}

/* ************************************************************************* */
// test Asia Joint
TEST(ADT, joint)
{
	DiscreteKey A(0, 2), S(1, 2), T(2, 2), L(3, 2), B(4, 2), E(5, 2), X(6, 2), D(7, 2);

	resetCounts();
	ADT pA = create(A % "99/1");
	ADT pS = create(S % "50/50");
	ADT pT = create(T | A = "99/1 95/5");
	ADT pL = create(L | S = "99/1 90/10");
	ADT pB = create(B | S = "70/30 40/60");
	ADT pE = create((E | T, L) = "F T T T");
	ADT pX = create(X | E = "95/5 2/98");
	ADT pD = create((D | E, B) = "9/1 2/8 3/7 1/9");
	printCounts("Asia CPTs");

	// Create joint
	resetCounts();
	ADT joint = pA;
	dot(joint, "Asia-A");
	joint = apply(joint, pS, &mul);
	dot(joint, "Asia-AS");
	joint = apply(joint, pT, &mul);
	dot(joint, "Asia-AST");
	joint = apply(joint, pL, &mul);
	dot(joint, "Asia-ASTL");
	joint = apply(joint, pB, &mul);
	dot(joint, "Asia-ASTLB");
	joint = apply(joint, pE, &mul);
	dot(joint, "Asia-ASTLBE");
	joint = apply(joint, pX, &mul);
	dot(joint, "Asia-ASTLBEX");
	joint = apply(joint, pD, &mul);
	dot(joint, "Asia-ASTLBEXD");
	EXPECT_LONGS_EQUAL(346, muls);
	printCounts("Asia joint");

	ADT pASTL = pA;
	pASTL = apply(pASTL, pS, &mul);
	pASTL = apply(pASTL, pT, &mul);
	pASTL = apply(pASTL, pL, &mul);

	// test combine
	ADT fAa = pASTL.combine(L, &add_).combine(T, &add_).combine(S, &add_);
	EXPECT(assert_equal(pA, fAa));
	ADT fAb = pASTL.combine(S, &add_).combine(T, &add_).combine(L, &add_);
	EXPECT(assert_equal(pA, fAb));
}

/* ************************************************************************* */
// test Inference with joint
TEST(ADT, inference)
{
	DiscreteKey A(0,2), D(1,2),//
			B(2,2), L(3,2), E(4,2), S(5,2), T(6,2), X(7,2);

	resetCounts();
	ADT pA = create(A % "99/1");
	ADT pS = create(S % "50/50");
	ADT pT = create(T | A = "99/1 95/5");
	ADT pL = create(L | S = "99/1 90/10");
	ADT pB = create(B | S = "70/30 40/60");
	ADT pE = create((E | T, L) = "F T T T");
	ADT pX = create(X | E = "95/5 2/98");
	ADT pD = create((D | E, B) = "9/1 2/8 3/7 1/9");
	//	printCounts("Inference CPTs");

	// Create joint
	resetCounts();
	ADT joint = pA;
	dot(joint, "Joint-Product-A");
	joint = apply(joint, pS, &mul);
	dot(joint, "Joint-Product-AS");
	joint = apply(joint, pT, &mul);
	dot(joint, "Joint-Product-AST");
	joint = apply(joint, pL, &mul);
	dot(joint, "Joint-Product-ASTL");
	joint = apply(joint, pB, &mul);
	dot(joint, "Joint-Product-ASTLB");
	joint = apply(joint, pE, &mul);
	dot(joint, "Joint-Product-ASTLBE");
	joint = apply(joint, pX, &mul);
	dot(joint, "Joint-Product-ASTLBEX");
	joint = apply(joint, pD, &mul);
	dot(joint, "Joint-Product-ASTLBEXD");
	EXPECT_LONGS_EQUAL(370, muls); // different ordering
	printCounts("Asia product");

	ADT marginal = joint;
	marginal = marginal.combine(X, &add_);
	dot(marginal, "Joint-Sum-ADBLEST");
	marginal = marginal.combine(T, &add_);
	dot(marginal, "Joint-Sum-ADBLES");
	marginal = marginal.combine(S, &add_);
	dot(marginal, "Joint-Sum-ADBLE");
	marginal = marginal.combine(E, &add_);
	dot(marginal, "Joint-Sum-ADBL");
	EXPECT_LONGS_EQUAL(161, adds);
	printCounts("Asia sum");
}

/* ************************************************************************* */
TEST(ADT, factor_graph)
{
	DiscreteKey B(0,2), L(1,2), E(2,2), S(3,2), T(4,2), X(5,2);

	resetCounts();
	ADT pS = create(S % "50/50");
	ADT pT = create(T % "95/5");
	ADT pL = create(L | S = "99/1 90/10");
	ADT pE = create((E | T, L) = "F T T T");
	ADT pX = create(X | E = "95/5 2/98");
	ADT pD = create(B | E = "1/8 7/9");
	ADT pB = create(B | S = "70/30 40/60");
	//	printCounts("Create CPTs");

	// Create joint
	resetCounts();
	ADT fg = pS;
	fg = apply(fg, pT, &mul);
	fg = apply(fg, pL, &mul);
	fg = apply(fg, pB, &mul);
	fg = apply(fg, pE, &mul);
	fg = apply(fg, pX, &mul);
	fg = apply(fg, pD, &mul);
	dot(fg, "FactorGraph");
	EXPECT_LONGS_EQUAL(158, muls);
	printCounts("Asia FG");

	fg = fg.combine(X, &add_);
	dot(fg, "Marginalized-6X");
	fg = fg.combine(T, &add_);
	dot(fg, "Marginalized-5T");
	fg = fg.combine(S, &add_);
	dot(fg, "Marginalized-4S");
	fg = fg.combine(E, &add_);
	dot(fg, "Marginalized-3E");
	fg = fg.combine(L, &add_);
	dot(fg, "Marginalized-2L");
	EXPECT(adds = 54);
	printCounts("marginalize");

	// BLESTX

	// Eliminate X
	ADT fE = pX;
	dot(fE, "Eliminate-01-fEX");
	fE = fE.combine(X, &add_);
	dot(fE, "Eliminate-02-fE");
	printCounts("Eliminate X");

	// Eliminate T
	ADT fLE = pT;
	fLE = apply(fLE, pE, &mul);
	dot(fLE, "Eliminate-03-fLET");
	fLE = fLE.combine(T, &add_);
	dot(fLE, "Eliminate-04-fLE");
	printCounts("Eliminate T");

	// Eliminate S
	ADT fBL = pS;
	fBL = apply(fBL, pL, &mul);
	fBL = apply(fBL, pB, &mul);
	dot(fBL, "Eliminate-05-fBLS");
	fBL = fBL.combine(S, &add_);
	dot(fBL, "Eliminate-06-fBL");
	printCounts("Eliminate S");

	// Eliminate E
	ADT fBL2 = fE;
	fBL2 = apply(fBL2, fLE, &mul);
	fBL2 = apply(fBL2, pD, &mul);
	dot(fBL2, "Eliminate-07-fBLE");
	fBL2 = fBL2.combine(E, &add_);
	dot(fBL2, "Eliminate-08-fBL2");
	printCounts("Eliminate E");

	// Eliminate L
	ADT fB = fBL;
	fB = apply(fB, fBL2, &mul);
	dot(fB, "Eliminate-09-fBL");
	fB = fB.combine(L, &add_);
	dot(fB, "Eliminate-10-fB");
	printCounts("Eliminate L");
}

/* ************************************************************************* */
// test equality
TEST(ADT, equality_noparser)
{
	DiscreteKey A(0,2), B(1,2);
	Signature::Table tableA, tableB;
	Signature::Row rA, rB;
	rA += 80, 20; rB += 60, 40;
	tableA += rA; tableB += rB;

	// Check straight equality
	ADT pA1 = create(A % tableA);
	ADT pA2 = create(A % tableA);
	EXPECT(pA1 == pA2); // should be equal

	// Check equality after apply
	ADT pB = create(B % tableB);
	ADT pAB1 = apply(pA1, pB, &mul);
	ADT pAB2 = apply(pB, pA1, &mul);
	EXPECT(pAB2 == pAB1);
}

/* ************************************************************************* */
// test equality
TEST(ADT, equality_parser)
{
	DiscreteKey A(0,2), B(1,2);
	// Check straight equality
	ADT pA1 = create(A % "80/20");
	ADT pA2 = create(A % "80/20");
	EXPECT(pA1 == pA2); // should be equal

	// Check equality after apply
	ADT pB = create(B % "60/40");
	ADT pAB1 = apply(pA1, pB, &mul);
	ADT pAB2 = apply(pB, pA1, &mul);
	EXPECT(pAB2 == pAB1);
}

/* ******************************************************************************** */
// Factor graph construction
// test constructor from strings
TEST(ADT, constructor)
{
	DiscreteKey v0(0,2), v1(1,3);
	Assignment<Index> x00, x01, x02, x10, x11, x12;
	x00[0] = 0, x00[1] = 0;
	x01[0] = 0, x01[1] = 1;
	x02[0] = 0, x02[1] = 2;
	x10[0] = 1, x10[1] = 0;
	x11[0] = 1, x11[1] = 1;
	x12[0] = 1, x12[1] = 2;

	ADT f1(v0 & v1, "0 1 2 3 4 5");
	EXPECT_DOUBLES_EQUAL(0, f1(x00), 1e-9);
	EXPECT_DOUBLES_EQUAL(1, f1(x01), 1e-9);
	EXPECT_DOUBLES_EQUAL(2, f1(x02), 1e-9);
	EXPECT_DOUBLES_EQUAL(3, f1(x10), 1e-9);
	EXPECT_DOUBLES_EQUAL(4, f1(x11), 1e-9);
	EXPECT_DOUBLES_EQUAL(5, f1(x12), 1e-9);

	ADT f2(v1 & v0, "0 1 2 3 4 5");
	EXPECT_DOUBLES_EQUAL(0, f2(x00), 1e-9);
	EXPECT_DOUBLES_EQUAL(2, f2(x01), 1e-9);
	EXPECT_DOUBLES_EQUAL(4, f2(x02), 1e-9);
	EXPECT_DOUBLES_EQUAL(1, f2(x10), 1e-9);
	EXPECT_DOUBLES_EQUAL(3, f2(x11), 1e-9);
	EXPECT_DOUBLES_EQUAL(5, f2(x12), 1e-9);

	DiscreteKey z0(0,5), z1(1,4), z2(2,3), z3(3,2);
	vector<double> table(5 * 4 * 3 * 2);
	double x = 0;
	BOOST_FOREACH(double& t, table)
	t = x++;
	ADT f3(z0 & z1 & z2 & z3, table);
	Assignment<Index> assignment;
	assignment[0] = 0;
	assignment[1] = 0;
	assignment[2] = 0;
	assignment[3] = 1;
	EXPECT_DOUBLES_EQUAL(1, f3(assignment), 1e-9);
}

/* ************************************************************************* */
// test conversion to integer indices
// Only works if DiscreteKeys are binary, as size_t has binary cardinality!
TEST(ADT, conversion)
{
	DiscreteKey X(0,2), Y(1,2);
	ADT fDiscreteKey(X & Y, "0.2 0.5 0.3 0.6");
	dot(fDiscreteKey, "conversion-f1");

	std::map<size_t, size_t> ordering;
	ordering[0] = 5;
	ordering[1] = 2;

	AlgebraicDecisionTree<size_t> fIndexKey(fDiscreteKey, ordering);
	//	f1.print("f1");
	//	f2.print("f2");
	dot(fIndexKey, "conversion-f2");

	Assignment<size_t> x00, x01, x02, x10, x11, x12;
	x00[5] = 0, x00[2] = 0;
	x01[5] = 0, x01[2] = 1;
	x10[5] = 1, x10[2] = 0;
	x11[5] = 1, x11[2] = 1;
	EXPECT_DOUBLES_EQUAL(0.2, fIndexKey(x00), 1e-9);
	EXPECT_DOUBLES_EQUAL(0.5, fIndexKey(x01), 1e-9);
	EXPECT_DOUBLES_EQUAL(0.3, fIndexKey(x10), 1e-9);
	EXPECT_DOUBLES_EQUAL(0.6, fIndexKey(x11), 1e-9);
}

/* ******************************************************************************** */
// test operations in elimination
TEST(ADT, elimination)
{
	DiscreteKey A(0,2), B(1,3), C(2,2);
	ADT f1(A & B & C, "1 2  3 4  5 6    1 8  3 3  5 5");
	dot(f1, "elimination-f1");

	{
		// sum out lower key
		ADT actualSum = f1.sum(C);
		ADT expectedSum(A & B, "3  7  11    9  6  10");
		CHECK(assert_equal(expectedSum,actualSum));

		// normalize
		ADT actual = f1 / actualSum;
		vector<double> cpt;
		cpt += 1.0 / 3, 2.0 / 3, 3.0 / 7, 4.0 / 7, 5.0 / 11, 6.0 / 11, //
		1.0 / 9, 8.0 / 9, 3.0 / 6, 3.0 / 6, 5.0 / 10, 5.0 / 10;
		ADT expected(A & B & C, cpt);
		CHECK(assert_equal(expected,actual));
	}

	{
		// sum out lower 2 keys
		ADT actualSum = f1.sum(C).sum(B);
		ADT expectedSum(A, 21, 25);
		CHECK(assert_equal(expectedSum,actualSum));

		// normalize
		ADT actual = f1 / actualSum;
		vector<double> cpt;
		cpt += 1.0 / 21, 2.0 / 21, 3.0 / 21, 4.0 / 21, 5.0 / 21, 6.0 / 21, //
		1.0 / 25, 8.0 / 25, 3.0 / 25, 3.0 / 25, 5.0 / 25, 5.0 / 25;
		ADT expected(A & B & C, cpt);
		CHECK(assert_equal(expected,actual));
	}
}

/* ******************************************************************************** */
// Test non-commutative op
TEST(ADT, div)
{
	DiscreteKey A(0,2), B(1,2);

	// Literals
	ADT a(A, 8, 16);
	ADT b(B, 2, 4);
	ADT expected_a_div_b(A & B, "4 2 8 4"); // 8/2 8/4 16/2 16/4
	ADT expected_b_div_a(A & B, "0.25 0.5 0.125 0.25"); // 2/8 4/8 2/16 4/16
	EXPECT(assert_equal(expected_a_div_b, a / b));
	EXPECT(assert_equal(expected_b_div_a, b / a));
}

/* ******************************************************************************** */
// test zero shortcut
TEST(ADT, zero)
{
	DiscreteKey A(0,2), B(1,2);

	// Literals
	ADT a(A, 0, 1);
	ADT notb(B, 1, 0);
	ADT anotb = a * notb;
	//	GTSAM_PRINT(anotb);
	Assignment<Index> x00, x01, x10, x11;
	x00[0] = 0, x00[1] = 0;
	x01[0] = 0, x01[1] = 1;
	x10[0] = 1, x10[1] = 0;
	x11[0] = 1, x11[1] = 1;
	EXPECT_DOUBLES_EQUAL(0, anotb(x00), 1e-9);
	EXPECT_DOUBLES_EQUAL(0, anotb(x01), 1e-9);
	EXPECT_DOUBLES_EQUAL(1, anotb(x10), 1e-9);
	EXPECT_DOUBLES_EQUAL(0, anotb(x11), 1e-9);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
