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

// #define DT_DEBUG_MEMORY
// #define DT_NO_PRUNING
#define DISABLE_DOT
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/discrete/Signature.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

template <typename T>
void dot(const T& f, const string& filename) {
#ifndef DISABLE_DOT
  f.dot(filename);
#endif
}

#define DOT(x) (dot(x, #x))

struct Crazy {
  int a;
  double b;
};

struct CrazyDecisionTree : public DecisionTree<string, Crazy> {
  /// print to stdout
  void print(const std::string& s = "") const {
    auto keyFormatter = [](const std::string& s) { return s; };
    auto valueFormatter = [](const Crazy& v) {
      return (boost::format("{%d,%4.2g}") % v.a % v.b).str();
    };
    DecisionTree<string, Crazy>::print("", keyFormatter, valueFormatter);
  }
  /// Equality method customized to Crazy node type
  bool equals(const CrazyDecisionTree& other, double tol = 1e-9) const {
    auto compare = [tol](const Crazy& v, const Crazy& w) {
      return v.a == w.a && std::abs(v.b - w.b) < tol;
    };
    return DecisionTree<string, Crazy>::equals(other, compare);
  }
};

// traits
namespace gtsam {
template <>
struct traits<CrazyDecisionTree> : public Testable<CrazyDecisionTree> {};
}  // namespace gtsam

GTSAM_CONCEPT_TESTABLE_INST(CrazyDecisionTree)

/* ************************************************************************** */
// Test string labels and int range
/* ************************************************************************** */

struct DT : public DecisionTree<string, int> {
  using Base = DecisionTree<string, int>;
  using DecisionTree::DecisionTree;
  DT() = default;

  DT(const Base& dt) : Base(dt) {}

  /// print to stdout
  void print(const std::string& s = "") const {
    auto keyFormatter = [](const std::string& s) { return s; };
    auto valueFormatter = [](const int& v) {
      return (boost::format("%d") % v).str();
    };
    Base::print("", keyFormatter, valueFormatter);
  }
  /// Equality method customized to int node type
  bool equals(const Base& other, double tol = 1e-9) const {
    auto compare = [](const int& v, const int& w) { return v == w; };
    return Base::equals(other, compare);
  }
};

// traits
namespace gtsam {
template <>
struct traits<DT> : public Testable<DT> {};
}  // namespace gtsam

GTSAM_CONCEPT_TESTABLE_INST(DT)

struct Ring {
  static inline int zero() { return 0; }
  static inline int one() { return 1; }
  static inline int id(const int& a) { return a; }
  static inline int add(const int& a, const int& b) { return a + b; }
  static inline int mul(const int& a, const int& b) { return a * b; }
};

/* ************************************************************************** */
// test DT
TEST(DecisionTree, example) {
  // Create labels
  string A("A"), B("B"), C("C");

  // create a value
  Assignment<string> x00, x01, x10, x11;
  x00[A] = 0, x00[B] = 0;
  x01[A] = 0, x01[B] = 1;
  x10[A] = 1, x10[B] = 0;
  x11[A] = 1, x11[B] = 1;

  // empty
  DT empty;

  // A
  DT a(A, 0, 5);
  LONGS_EQUAL(0, a(x00))
  LONGS_EQUAL(5, a(x10))
  DOT(a);

  // pruned
  DT p(A, 2, 2);
  LONGS_EQUAL(2, p(x00))
  LONGS_EQUAL(2, p(x10))
  DOT(p);

  // \neg B
  DT notb(B, 5, 0);
  LONGS_EQUAL(5, notb(x00))
  LONGS_EQUAL(5, notb(x10))
  DOT(notb);

  // Check supplying empty trees yields an exception
  CHECK_EXCEPTION(gtsam::apply(empty, &Ring::id), std::runtime_error);
  CHECK_EXCEPTION(gtsam::apply(empty, a, &Ring::mul), std::runtime_error);
  CHECK_EXCEPTION(gtsam::apply(a, empty, &Ring::mul), std::runtime_error);

  // apply, two nodes, in natural order
  DT anotb = apply(a, notb, &Ring::mul);
  LONGS_EQUAL(0, anotb(x00))
  LONGS_EQUAL(0, anotb(x01))
  LONGS_EQUAL(25, anotb(x10))
  LONGS_EQUAL(0, anotb(x11))
  DOT(anotb);

  // check pruning
  DT pnotb = apply(p, notb, &Ring::mul);
  LONGS_EQUAL(10, pnotb(x00))
  LONGS_EQUAL(0, pnotb(x01))
  LONGS_EQUAL(10, pnotb(x10))
  LONGS_EQUAL(0, pnotb(x11))
  DOT(pnotb);

  // check pruning
  DT zeros = apply(DT(A, 0, 0), notb, &Ring::mul);
  LONGS_EQUAL(0, zeros(x00))
  LONGS_EQUAL(0, zeros(x01))
  LONGS_EQUAL(0, zeros(x10))
  LONGS_EQUAL(0, zeros(x11))
  DOT(zeros);

  // apply, two nodes, in switched order
  DT notba = apply(a, notb, &Ring::mul);
  LONGS_EQUAL(0, notba(x00))
  LONGS_EQUAL(0, notba(x01))
  LONGS_EQUAL(25, notba(x10))
  LONGS_EQUAL(0, notba(x11))
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
  LONGS_EQUAL(0, a_and_a(x00))
  LONGS_EQUAL(0, a_and_a(x01))
  LONGS_EQUAL(25, a_and_a(x10))
  LONGS_EQUAL(25, a_and_a(x11))
  DOT(a_and_a);

  // create a function on C
  DT c(C, 0, 5);

  // and a model assigning stuff to C
  Assignment<string> x101;
  x101[A] = 1, x101[B] = 0, x101[C] = 1;

  // mul notba with C
  DT notbac = apply(notba, c, &Ring::mul);
  LONGS_EQUAL(125, notbac(x101))
  DOT(notbac);

  // mul now in different order
  DT acnotb = apply(apply(a, c, &Ring::mul), notb, &Ring::mul);
  LONGS_EQUAL(125, acnotb(x101))
  DOT(acnotb);
}

/* ************************************************************************** */
// test Conversion of values
bool bool_of_int(const int& y) { return y != 0; };
typedef DecisionTree<string, bool> StringBoolTree;

TEST(DecisionTree, ConvertValuesOnly) {
  // Create labels
  string A("A"), B("B");

  // apply, two nodes, in natural order
  DT f1 = apply(DT(A, 0, 5), DT(B, 5, 0), &Ring::mul);

  // convert
  StringBoolTree f2(f1, bool_of_int);

  // Check a value
  Assignment<string> x00;
  x00["A"] = 0, x00["B"] = 0;
  EXPECT(!f2(x00));
}

/* ************************************************************************** */
// test Conversion of both values and labels.
enum Label { U, V, X, Y, Z };
typedef DecisionTree<Label, bool> LabelBoolTree;

TEST(DecisionTree, ConvertBoth) {
  // Create labels
  string A("A"), B("B");

  // apply, two nodes, in natural order
  DT f1 = apply(DT(A, 0, 5), DT(B, 5, 0), &Ring::mul);

  // convert
  map<string, Label> ordering;
  ordering[A] = X;
  ordering[B] = Y;
  LabelBoolTree f2(f1, ordering, &bool_of_int);

  // Check some values
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

/* ************************************************************************** */
// test Compose expansion
TEST(DecisionTree, Compose) {
  // Create labels
  string A("A"), B("B"), C("C");

  // Put two stumps on A together
  DT f1(B, DT(A, 0, 1), DT(A, 2, 3));

  // Create from string
  vector<DT::LabelC> keys;
  keys += DT::LabelC(A, 2), DT::LabelC(B, 2);
  DT f2(keys, "0 2 1 3");
  EXPECT(assert_equal(f2, f1, 1e-9));

  // Put this AB tree together with another one
  DT f3(keys, "4 6 5 7");
  DT f4(C, f1, f3);
  DOT(f4);

  // a bigger tree
  keys += DT::LabelC(C, 2);
  DT f5(keys, "0 4 2 6 1 5 3 7");
  EXPECT(assert_equal(f5, f4, 1e-9));
  DOT(f5);
}

/* ************************************************************************** */
// Check we can create a decision tree of containers.
TEST(DecisionTree, Containers) {
  using Container = std::vector<double>;
  using StringContainerTree = DecisionTree<string, Container>;

  // Check default constructor
  StringContainerTree tree;

  // Create small two-level tree
  string A("A"), B("B");
  DT stringIntTree(B, DT(A, 0, 1), DT(A, 2, 3));

  // Check conversion
  auto container_of_int = [](const int& i) {
    Container c;
    c.emplace_back(i);
    return c;
  };
  StringContainerTree converted(stringIntTree, container_of_int);
}

/* ************************************************************************** */
// Test visit.
TEST(DecisionTree, visit) {
  // Create small two-level tree
  string A("A"), B("B");
  DT tree(B, DT(A, 0, 1), DT(A, 2, 3));
  double sum = 0.0;
  auto visitor = [&](int y) { sum += y; };
  tree.visit(visitor);
  EXPECT_DOUBLES_EQUAL(6.0, sum, 1e-9);
}

/* ************************************************************************** */
// Test visit, with Choices argument.
TEST(DecisionTree, visitWith) {
  // Create small two-level tree
  string A("A"), B("B");
  DT tree(B, DT(A, 0, 1), DT(A, 2, 3));
  double sum = 0.0;
  auto visitor = [&](const Assignment<string>& choices, int y) { sum += y; };
  tree.visitWith(visitor);
  EXPECT_DOUBLES_EQUAL(6.0, sum, 1e-9);
}

/* ************************************************************************** */
// Test fold.
TEST(DecisionTree, fold) {
  // Create small two-level tree
  string A("A"), B("B");
  DT tree(B, DT(A, 1, 1), DT(A, 2, 3));
  auto add = [](const int& y, double x) { return y + x; };
  double sum = tree.fold(add, 0.0);
  EXPECT_DOUBLES_EQUAL(6.0, sum, 1e-9);  // Note, not 7, due to pruning!
}

/* ************************************************************************** */
// Test retrieving all labels.
TEST(DecisionTree, labels) {
  // Create small two-level tree
  string A("A"), B("B");
  DT tree(B, DT(A, 0, 1), DT(A, 2, 3));
  auto labels = tree.labels();
  EXPECT_LONGS_EQUAL(2, labels.size());
}

/* ************************************************************************** */
// Test unzip method.
TEST(DecisionTree, unzip) {
  using DTP = DecisionTree<string, std::pair<int, string>>;
  using DT1 = DecisionTree<string, int>;
  using DT2 = DecisionTree<string, string>;

  // Create small two-level tree
  string A("A"), B("B"), C("C");
  DTP tree(B, DTP(A, {0, "zero"}, {1, "one"}),
           DTP(A, {2, "two"}, {1337, "l33t"}));

  DT1 dt1;
  DT2 dt2;
  std::tie(dt1, dt2) = unzip(tree);

  DT1 tree1(B, DT1(A, 0, 1), DT1(A, 2, 1337));
  DT2 tree2(B, DT2(A, "zero", "one"), DT2(A, "two", "l33t"));

  EXPECT(tree1.equals(dt1));
  EXPECT(tree2.equals(dt2));
}

/* ************************************************************************** */
// Test thresholding.
TEST(DecisionTree, threshold) {
  // Create three level tree
  vector<DT::LabelC> keys;
  keys += DT::LabelC("C", 2), DT::LabelC("B", 2), DT::LabelC("A", 2);
  DT tree(keys, "0 1 2 3 4 5 6 7");

  // Check number of leaves equal to zero
  auto count = [](const int& value, int count) {
    return value == 0 ? count + 1 : count;
  };
  EXPECT_LONGS_EQUAL(1, tree.fold(count, 0));

  // Now threshold
  auto threshold = [](int value) { return value < 5 ? 0 : value; };
  DT thresholded(tree, threshold);

  // Check number of leaves equal to zero now = 2
  // Note: it is 2, because the pruned branches are counted as 1!
  EXPECT_LONGS_EQUAL(2, thresholded.fold(count, 0));
}

/* ************************************************************************** */
// Test convertFromWithChoice constructor.
TEST(DecisionTree, ConvertFromWithChoice) {
  // Create three level tree
  vector<DT::LabelC> keys;
  keys += DT::LabelC("C", 2), DT::LabelC("B", 2), DT::LabelC("A", 2);
  DT tree(keys, "0 1 2 3 4 5 6 7");

  DT predicateTree(keys, "0 1 2 -1 4 5 -1 -1");

  std::vector<int> nodes;
  auto collector = [&](const int& x) {
    nodes.push_back(x);
  };
  predicateTree.print();
  predicateTree.visit(collector);
  std::cout << "Number of node (should be < 8): " << nodes.size() << std::endl;

  // This constructor will fail because the last 2 -1s are merged together.
  DT actualTree(keys, nodes);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
