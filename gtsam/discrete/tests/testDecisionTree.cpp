/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDecisionTree.cpp
 * @brief    Develop DecisionTree
 * @author  Frank Dellaert
 * @author  Can Erdogan
 * @date    Jan 30, 2012
 */

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/Signature.h>

//#define DT_DEBUG_MEMORY
//#define DT_NO_PRUNING
#define DISABLE_DOT
#include <gtsam/discrete/DecisionTree-inl.h>
using namespace std;
using namespace gtsam;

template<typename T>
void dot(const T&f, const string& filename) {
#ifndef DISABLE_DOT
  f.dot(filename);
#endif
}

#define DOT(x)(dot(x,#x))

struct Crazy { int a; double b; };
typedef DecisionTree<string,Crazy> CrazyDecisionTree; // check that DecisionTree is actually generic (as it pretends to be)

// traits
namespace gtsam {
template<> struct traits<CrazyDecisionTree> : public Testable<CrazyDecisionTree> {};
}

/* ******************************************************************************** */
// Test string labels and int range
/* ******************************************************************************** */

typedef DecisionTree<string, int> DT;

// traits
namespace gtsam {
template<> struct traits<DT> : public Testable<DT> {};
}

struct Ring {
  static inline int zero() {
    return 0;
  }
  static inline int one() {
    return 1;
  }
  static inline int add(const int& a, const int& b) {
    return a + b;
  }
  static inline int mul(const int& a, const int& b) {
    return a * b;
  }
};

/* ******************************************************************************** */
// test DT
TEST(DT, example)
{
  // Create labels
  string A("A"), B("B"), C("C");

  // create a value
  Assignment<string> x00, x01, x10, x11;
  x00[A] = 0, x00[B] = 0;
  x01[A] = 0, x01[B] = 1;
  x10[A] = 1, x10[B] = 0;
  x11[A] = 1, x11[B] = 1;

  // A
  DT a(A, 0, 5);
  LONGS_EQUAL(0,a(x00))
  LONGS_EQUAL(5,a(x10))
  DOT(a);

  // pruned
  DT p(A, 2, 2);
  LONGS_EQUAL(2,p(x00))
  LONGS_EQUAL(2,p(x10))
  DOT(p);

  // \neg B
  DT notb(B, 5, 0);
  LONGS_EQUAL(5,notb(x00))
  LONGS_EQUAL(5,notb(x10))
  DOT(notb);

  // apply, two nodes, in natural order
  DT anotb = apply(a, notb, &Ring::mul);
  LONGS_EQUAL(0,anotb(x00))
  LONGS_EQUAL(0,anotb(x01))
  LONGS_EQUAL(25,anotb(x10))
  LONGS_EQUAL(0,anotb(x11))
  DOT(anotb);

  // check pruning
  DT pnotb = apply(p, notb, &Ring::mul);
  LONGS_EQUAL(10,pnotb(x00))
  LONGS_EQUAL( 0,pnotb(x01))
  LONGS_EQUAL(10,pnotb(x10))
  LONGS_EQUAL( 0,pnotb(x11))
  DOT(pnotb);

  // check pruning
  DT zeros = apply(DT(A, 0, 0), notb, &Ring::mul);
  LONGS_EQUAL(0,zeros(x00))
  LONGS_EQUAL(0,zeros(x01))
  LONGS_EQUAL(0,zeros(x10))
  LONGS_EQUAL(0,zeros(x11))
  DOT(zeros);

  // apply, two nodes, in switched order
  DT notba = apply(a, notb, &Ring::mul);
  LONGS_EQUAL(0,notba(x00))
  LONGS_EQUAL(0,notba(x01))
  LONGS_EQUAL(25,notba(x10))
  LONGS_EQUAL(0,notba(x11))
  DOT(notba);

  // Test choose 0
  DT actual0 = notba.choose(A, 0);
  EXPECT(assert_equal(DT(0.0), actual0));
  DOT(actual0);

  // Test choose 1
  DT actual1 = notba.choose(A, 1);
  EXPECT(assert_equal(DT(B, 25, 0), actual1));
  DOT(actual1);

  // apply, two nodes at same level
  DT a_and_a = apply(a, a, &Ring::mul);
  LONGS_EQUAL(0,a_and_a(x00))
  LONGS_EQUAL(0,a_and_a(x01))
  LONGS_EQUAL(25,a_and_a(x10))
  LONGS_EQUAL(25,a_and_a(x11))
  DOT(a_and_a);

  // create a function on C
  DT c(C, 0, 5);

  // and a model assigning stuff to C
  Assignment<string> x101;
  x101[A] = 1, x101[B] = 0, x101[C] = 1;

  // mul notba with C
  DT notbac = apply(notba, c, &Ring::mul);
  LONGS_EQUAL(125,notbac(x101))
  DOT(notbac);

  // mul now in different order
  DT acnotb = apply(apply(a, c, &Ring::mul), notb, &Ring::mul);
  LONGS_EQUAL(125,acnotb(x101))
  DOT(acnotb);
}

/* ******************************************************************************** */
// test Conversion
enum Label {
  U, V, X, Y, Z
};
typedef DecisionTree<Label, bool> BDT;
bool convert(const int& y) {
  return y != 0;
}

TEST(DT, conversion)
{
  // Create labels
  string A("A"), B("B");

  // apply, two nodes, in natural order
  DT f1 = apply(DT(A, 0, 5), DT(B, 5, 0), &Ring::mul);

  // convert
  map<string, Label> ordering;
  ordering[A] = X;
  ordering[B] = Y;
  boost::function<bool(const int&)> op = convert;
  BDT f2(f1, ordering, op);
  //  f1.print("f1");
  //  f2.print("f2");

  // create a value
  Assignment<Label> x00, x01, x10, x11;
  x00[X] = 0, x00[Y] = 0;
  x01[X] = 0, x01[Y] = 1;
  x10[X] = 1, x10[Y] = 0;
  x11[X] = 1, x11[Y] = 1;
  EXPECT(!f2(x00));
  EXPECT(!f2(x01));
  EXPECT(f2(x10));
  EXPECT(!f2(x11));
}

/* ******************************************************************************** */
// test Compose expansion
TEST(DT, Compose)
{
  // Create labels
  string A("A"), B("B"), C("C");

  // Put two stumps on A together
  DT f1(B, DT(A, 0, 1), DT(A, 2, 3));

  // Create from string
  vector<DT::LabelC> keys;
  keys += DT::LabelC(A,2), DT::LabelC(B,2);
  DT f2(keys, "0 2 1 3");
  EXPECT(assert_equal(f2, f1, 1e-9));

  // Put this AB tree together with another one
  DT f3(keys, "4 6 5 7");
  DT f4(C, f1, f3);
  DOT(f4);

  // a bigger tree
  keys += DT::LabelC(C,2);
  DT f5(keys, "0 4 2 6 1 5 3 7");
  EXPECT(assert_equal(f5, f4, 1e-9));
  DOT(f5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
