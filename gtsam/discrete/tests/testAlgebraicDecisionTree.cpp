/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file   testAlgebraicDecisionTree.cpp
 * @brief  Unit tests for Algebraic decision tree
 * @author Frank Dellaert
 * @date   Mar 6, 2011
 */

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteKey.h>  // make sure we have traits
#include <gtsam/discrete/DiscreteValues.h>
// headers first to make sure no missing headers
#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DecisionTree-inl.h>  // for convert only
#include <gtsam/discrete/Signature.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************** */
typedef AlgebraicDecisionTree<Key> ADT;

// traits
namespace gtsam {
template <>
struct traits<ADT> : public Testable<ADT> {};
}  // namespace gtsam

#define DISABLE_DOT

template <typename T>
void dot(const T& f, const string& filename) {
#ifndef DISABLE_DOT
  f.dot(filename);
#endif
}

/* ************************************************************************** */
// Test arithmetic:
TEST(ADT, arithmetic) {
  DiscreteKey A(0, 2), B(1, 2);
  ADT zero{0}, one{1};
  ADT a(A, 1, 2);
  ADT b(B, 3, 4);

  // Addition
  CHECK(assert_equal(a, zero + a));

  // Negate and subtraction
  CHECK(assert_equal(-a, zero - a));
  CHECK(assert_equal({zero}, a - a));
  CHECK(assert_equal(a + b, b + a));
  CHECK(assert_equal({A, 3, 4}, a + 2));
  CHECK(assert_equal({B, 1, 2}, b - 2));

  // Multiplication
  CHECK(assert_equal(zero, zero * a));
  CHECK(assert_equal(zero, a * zero));
  CHECK(assert_equal(a, one * a));
  CHECK(assert_equal(a, a * one));
  CHECK(assert_equal(a * b, b * a));

  // division
  // CHECK(assert_equal(a, (a * b) / b)); // not true because no pruning
  CHECK(assert_equal(b, (a * b) / a));
}

/* ************************************************************************** */
// instrumented operators
/* ************************************************************************** */
size_t muls = 0, adds = 0;
void resetCounts() {
  muls = 0;
  adds = 0;
}
void printCounts(const string& s) {
#ifndef DISABLE_TIMING
  cout << s << ": " << std::setw(3) << muls << " muls, " << std::setw(3) << adds
       << " adds" << endl;
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

/* ************************************************************************** */
// test ADT
TEST(ADT, example3) {
  // Create labels
  DiscreteKey A(0, 2), B(1, 2), C(2, 2), D(3, 2), E(4, 2);

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
  //  acnotb.print("acnotb: ");
  //  acnotb.printCache("acnotb Cache:");

  dot(acnotb, "ADT-acnotb");

  ADT big = apply(apply(d, note, &mul), acnotb, &add_);
  dot(big, "ADT-big");
}

/* ************************************************************************** */
// Asia Bayes Network
/* ************************************************************************** */

/** Convert Signature into CPT */
ADT create(const Signature& signature) {
  ADT p(signature.discreteKeys(), signature.cpt());
  static size_t count = 0;
  const DiscreteKey& key = signature.key();
  std::stringstream ss;
  ss << "CPT-" << std::setw(3) << std::setfill('0') << ++count << "-"
     << key.first;
  string DOTfile = ss.str();
  dot(p, DOTfile);
  return p;
}

/* ************************************************************************* */
namespace asiaCPTs {
DiscreteKey A(0, 2), S(1, 2), T(2, 2), L(3, 2), B(4, 2), E(5, 2), X(6, 2),
    D(7, 2);

ADT pA = create(A % "99/1");
ADT pS = create(S % "50/50");
ADT pT = create(T | A = "99/1 95/5");
ADT pL = create(L | S = "99/1 90/10");
ADT pB = create(B | S = "70/30 40/60");
ADT pE = create((E | T, L) = "F T T T");
ADT pX = create(X | E = "95/5 2/98");
ADT pD = create((D | E, B) = "9/1 2/8 3/7 1/9");
}  // namespace asiaCPTs

/* ************************************************************************* */
// test Asia Joint
TEST(ADT, joint) {
  using namespace asiaCPTs;

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
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(346, muls);
#else
  EXPECT_LONGS_EQUAL(508, muls);
#endif
  printCounts("Asia joint");
}

/* ************************************************************************* */
TEST(ADT, combine) {
  using namespace asiaCPTs;

  // Form P(A,S,T,L) = P(A) P(S) P(T|A) P(L|S)
  ADT pASTL = pA;
  pASTL = apply(pASTL, pS, &mul);
  pASTL = apply(pASTL, pT, &mul);
  pASTL = apply(pASTL, pL, &mul);

  // test combine to check that P(A) = \sum_{S,T,L} P(A,S,T,L)
  ADT fAa = pASTL.combine(L, &add_).combine(T, &add_).combine(S, &add_);
  EXPECT(assert_equal(pA, fAa));
  ADT fAb = pASTL.combine(S, &add_).combine(T, &add_).combine(L, &add_);
  EXPECT(assert_equal(pA, fAb));
}

/* ************************************************************************* */
// test Inference with joint, created using different ordering
TEST(ADT, inference) {
  DiscreteKey A(0, 2), D(1, 2),  //
      B(2, 2), L(3, 2), E(4, 2), S(5, 2), T(6, 2), X(7, 2);

  ADT pA = create(A % "99/1");
  ADT pS = create(S % "50/50");
  ADT pT = create(T | A = "99/1 95/5");
  ADT pL = create(L | S = "99/1 90/10");
  ADT pB = create(B | S = "70/30 40/60");
  ADT pE = create((E | T, L) = "F T T T");
  ADT pX = create(X | E = "95/5 2/98");
  ADT pD = create((D | E, B) = "9/1 2/8 3/7 1/9");

  // Create joint, note different ordering than above: different tree!
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
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(370, (long)muls);  // different ordering
#else
  EXPECT_LONGS_EQUAL(508, (long)muls);  // different ordering
#endif
  printCounts("Asia product");

  resetCounts();
  ADT marginal = joint;
  marginal = marginal.combine(X, &add_);
  dot(marginal, "Joint-Sum-ADBLEST");
  marginal = marginal.combine(T, &add_);
  dot(marginal, "Joint-Sum-ADBLES");
  marginal = marginal.combine(S, &add_);
  dot(marginal, "Joint-Sum-ADBLE");
  marginal = marginal.combine(E, &add_);
  dot(marginal, "Joint-Sum-ADBL");
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(161, (long)adds);
#else
  EXPECT_LONGS_EQUAL(240, (long)adds);
#endif
  printCounts("Asia sum");
}

/* ************************************************************************* */
TEST(ADT, factor_graph) {
  DiscreteKey B(0, 2), L(1, 2), E(2, 2), S(3, 2), T(4, 2), X(5, 2);

  ADT pS = create(S % "50/50");
  ADT pT = create(T % "95/5");
  ADT pL = create(L | S = "99/1 90/10");
  ADT pE = create((E | T, L) = "F T T T");
  ADT pX = create(X | E = "95/5 2/98");
  ADT pD = create(B | E = "1/8 7/9");
  ADT pB = create(B | S = "70/30 40/60");

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
#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(158, (long)muls);
#else
  EXPECT_LONGS_EQUAL(188, (long)muls);
#endif
  printCounts("Asia FG");

  resetCounts();
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
#ifdef GTSAM_DT_MERGING
  LONGS_EQUAL(49, adds);
#else
  LONGS_EQUAL(62, adds);
#endif
  printCounts("marginalize");

  // BLESTX

  // Eliminate X
  resetCounts();
  ADT fE = pX;
  dot(fE, "Eliminate-01-fEX");
  fE = fE.combine(X, &add_);
  dot(fE, "Eliminate-02-fE");
  printCounts("Eliminate X");

  // Eliminate T
  resetCounts();
  ADT fLE = pT;
  fLE = apply(fLE, pE, &mul);
  dot(fLE, "Eliminate-03-fLET");
  fLE = fLE.combine(T, &add_);
  dot(fLE, "Eliminate-04-fLE");
  printCounts("Eliminate T");

  // Eliminate S
  resetCounts();
  ADT fBL = pS;
  fBL = apply(fBL, pL, &mul);
  fBL = apply(fBL, pB, &mul);
  dot(fBL, "Eliminate-05-fBLS");
  fBL = fBL.combine(S, &add_);
  dot(fBL, "Eliminate-06-fBL");
  printCounts("Eliminate S");

  // Eliminate E
  resetCounts();
  ADT fBL2 = fE;
  fBL2 = apply(fBL2, fLE, &mul);
  fBL2 = apply(fBL2, pD, &mul);
  dot(fBL2, "Eliminate-07-fBLE");
  fBL2 = fBL2.combine(E, &add_);
  dot(fBL2, "Eliminate-08-fBL2");
  printCounts("Eliminate E");

  // Eliminate L
  resetCounts();
  ADT fB = fBL;
  fB = apply(fB, fBL2, &mul);
  dot(fB, "Eliminate-09-fBL");
  fB = fB.combine(L, &add_);
  dot(fB, "Eliminate-10-fB");
  printCounts("Eliminate L");
}

/* ************************************************************************* */
// test equality
TEST(ADT, equality_noparser) {
  const DiscreteKey A(0, 2), B(1, 2);
  const Signature::Row rA{80, 20}, rB{60, 40};
  const Signature::Table tableA{rA}, tableB{rB};

  // Check straight equality
  ADT pA1 = create(A % tableA);
  ADT pA2 = create(A % tableA);
  EXPECT(pA1.equals(pA2));  // should be equal

  // Check equality after apply
  ADT pB = create(B % tableB);
  ADT pAB1 = apply(pA1, pB, &mul);
  ADT pAB2 = apply(pB, pA1, &mul);
  EXPECT(pAB2.equals(pAB1));
}

/* ************************************************************************* */
// test equality
TEST(ADT, equality_parser) {
  DiscreteKey A(0, 2), B(1, 2);
  // Check straight equality
  ADT pA1 = create(A % "80/20");
  ADT pA2 = create(A % "80/20");
  EXPECT(pA1.equals(pA2));  // should be equal

  // Check equality after apply
  ADT pB = create(B % "60/40");
  ADT pAB1 = apply(pA1, pB, &mul);
  ADT pAB2 = apply(pB, pA1, &mul);
  EXPECT(pAB2.equals(pAB1));
}

/* ************************************************************************** */
// Factor graph construction
// test constructor from strings
TEST(ADT, constructor) {
  DiscreteKey v0(0, 2), v1(1, 3);
  DiscreteValues x00, x01, x02, x10, x11, x12;
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

  DiscreteKey z0(0, 5), z1(1, 4), z2(2, 3), z3(3, 2);
  vector<double> table(5 * 4 * 3 * 2);
  double x = 0;
  for (double& t : table) t = x++;
  ADT f3(z0 & z1 & z2 & z3, table);
  DiscreteValues assignment;
  assignment[0] = 0;
  assignment[1] = 0;
  assignment[2] = 0;
  assignment[3] = 1;
  EXPECT_DOUBLES_EQUAL(1, f3(assignment), 1e-9);
}

/* ************************************************************************* */
// test conversion to integer indices
// Only works if DiscreteKeys are binary, as size_t has binary cardinality!
TEST(ADT, conversion) {
  DiscreteKey X(0, 2), Y(1, 2);
  ADT fDiscreteKey(X & Y, "0.2 0.5 0.3 0.6");
  dot(fDiscreteKey, "conversion-f1");

  std::map<Key, Key> keyMap;
  keyMap[0] = 5;
  keyMap[1] = 2;

  AlgebraicDecisionTree<Key> fIndexKey(fDiscreteKey, keyMap);
  //  f1.print("f1");
  //  f2.print("f2");
  dot(fIndexKey, "conversion-f2");

  DiscreteValues x00, x01, x02, x10, x11, x12;
  x00[5] = 0, x00[2] = 0;
  x01[5] = 0, x01[2] = 1;
  x10[5] = 1, x10[2] = 0;
  x11[5] = 1, x11[2] = 1;
  EXPECT_DOUBLES_EQUAL(0.2, fIndexKey(x00), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.5, fIndexKey(x01), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.3, fIndexKey(x10), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.6, fIndexKey(x11), 1e-9);
}

/* ************************************************************************** */
// test operations in elimination
TEST(ADT, elimination) {
  DiscreteKey A(0, 2), B(1, 3), C(2, 2);
  ADT f1(A & B & C, "1 2  3 4  5 6    1 8  3 3  5 5");
  dot(f1, "elimination-f1");

  {
    // sum out lower key
    ADT actualSum = f1.sum(C);
    ADT expectedSum(A & B, "3  7  11    9  6  10");
    CHECK(assert_equal(expectedSum, actualSum));

    // normalize
    ADT actual = f1 / actualSum;
    const vector<double> cpt{
        1.0 / 3, 2.0 / 3, 3.0 / 7, 4.0 / 7, 5.0 / 11, 6.0 / 11,  //
        1.0 / 9, 8.0 / 9, 3.0 / 6, 3.0 / 6, 5.0 / 10, 5.0 / 10};
    ADT expected(A & B & C, cpt);
    CHECK(assert_equal(expected, actual));
  }

  {
    // sum out lower 2 keys
    ADT actualSum = f1.sum(C).sum(B);
    ADT expectedSum(A, 21, 25);
    CHECK(assert_equal(expectedSum, actualSum));

    // normalize
    ADT actual = f1 / actualSum;
    const vector<double> cpt{
        1.0 / 21, 2.0 / 21, 3.0 / 21, 4.0 / 21, 5.0 / 21, 6.0 / 21,  //
        1.0 / 25, 8.0 / 25, 3.0 / 25, 3.0 / 25, 5.0 / 25, 5.0 / 25};
    ADT expected(A & B & C, cpt);
    CHECK(assert_equal(expected, actual));
  }
}

/* ************************************************************************** */
// Test non-commutative op
TEST(ADT, div) {
  DiscreteKey A(0, 2), B(1, 2);

  // Literals
  ADT a(A, 8, 16);
  ADT b(B, 2, 4);
  ADT expected_a_div_b(A & B, "4 2 8 4");              // 8/2 8/4 16/2 16/4
  ADT expected_b_div_a(A & B, "0.25 0.5 0.125 0.25");  // 2/8 4/8 2/16 4/16
  EXPECT(assert_equal(expected_a_div_b, a / b));
  EXPECT(assert_equal(expected_b_div_a, b / a));
}

/* ************************************************************************** */
// test zero shortcut
TEST(ADT, zero) {
  DiscreteKey A(0, 2), B(1, 2);

  // Literals
  ADT a(A, 0, 1);
  ADT notb(B, 1, 0);
  ADT anotb = a * notb;
  //  GTSAM_PRINT(anotb);
  DiscreteValues x00, x01, x10, x11;
  x00[0] = 0, x00[1] = 0;
  x01[0] = 0, x01[1] = 1;
  x10[0] = 1, x10[1] = 0;
  x11[0] = 1, x11[1] = 1;
  EXPECT_DOUBLES_EQUAL(0, anotb(x00), 1e-9);
  EXPECT_DOUBLES_EQUAL(0, anotb(x01), 1e-9);
  EXPECT_DOUBLES_EQUAL(1, anotb(x10), 1e-9);
  EXPECT_DOUBLES_EQUAL(0, anotb(x11), 1e-9);
}

/// Example ADT from 0 to 11.
ADT exampleADT() {
  DiscreteKey A(0, 2), B(1, 3), C(2, 2);
  ADT f(A & B & C, "0 6  2 8  4 10    1 7  3 9  5 11");
  return f;
}
/* ************************************************************************** */
// Test sum
TEST(ADT, Sum) {
  ADT a = exampleADT();
  double expected_sum = 0;
  for (double i = 0; i < 12; i++) {
    expected_sum += i;
  }
  EXPECT_DOUBLES_EQUAL(expected_sum, a.sum(), 1e-9);
}

/* ************************************************************************** */
// Test normalize
TEST(ADT, Normalize) {
  ADT a = exampleADT();
  double sum = a.sum();
  auto actual = a.normalize();

  DiscreteKey A(0, 2), B(1, 3), C(2, 2);
  DiscreteKeys keys = DiscreteKeys{A, B, C};
  std::vector<double> cpt{0 / sum, 6 / sum,  2 / sum, 8 / sum,
                          4 / sum, 10 / sum, 1 / sum, 7 / sum,
                          3 / sum, 9 / sum,  5 / sum, 11 / sum};
  ADT expected(keys, cpt);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************** */
// Test min
TEST(ADT, Min) {
  ADT a = exampleADT();
  double min = a.min();
  EXPECT_DOUBLES_EQUAL(0.0, min, 1e-9);
}

/* ************************************************************************** */
// Test max
TEST(ADT, Max) {
  ADT a = exampleADT();
  double max = a.max();
  EXPECT_DOUBLES_EQUAL(11.0, max, 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
