/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file testDiscreteFactorGraph.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::M;

/* ************************************************************************* */
TEST_UNSAFE(DiscreteFactorGraph, debugScheduler) {
  DiscreteKey PC(0, 4), ME(1, 4), AI(2, 4), A(3, 3);

  DiscreteFactorGraph graph;
  graph.add(AI, "1 0 0 1");
  graph.add(AI, "1 1 1 0");
  graph.add(A & AI, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(ME, "0 1 0 0");
  graph.add(ME, "1 1 1 0");
  graph.add(A & ME, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(PC, "1 0 1 0");
  graph.add(PC, "1 1 1 0");
  graph.add(A & PC, "1 1 1 0   1 1 1 1   0 1 1 1");
  graph.add(ME & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");
  graph.add(PC & ME, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");
  graph.add(PC & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0");

  // Check MPE.
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal({{0, 2}, {1, 1}, {2, 0}, {3, 0}}, actualMPE));
}

/* ************************************************************************* */
/// Test the () operator of DiscreteFactorGraph
TEST_UNSAFE( DiscreteFactorGraph, DiscreteFactorGraphEvaluationTest) {

  // Three keys P1 and P2
  DiscreteKey P1(0,2), P2(1,2), P3(2,3);

  // Create the DiscreteFactorGraph
  DiscreteFactorGraph graph;
  graph.add(P1, "0.9 0.3");
  graph.add(P2, "0.9 0.6");
  graph.add(P1 & P2, "4 1 10 4");

  // Instantiate DiscreteValues
  DiscreteValues values;
  values[0] = 1;
  values[1] = 1;

  // Check if graph evaluation works ( 0.3*0.6*4 )
  EXPECT_DOUBLES_EQUAL( .72, graph(values), 1e-9);

  // Creating a new test with third node and adding unary and ternary factors on it
  graph.add(P3, "0.9 0.2 0.5");
  graph.add(P1 & P2 & P3, "1 2 3 4 5 6 7 8 9 10 11 12");

  // Below values lead to selecting the 8th index in the ternary factor table
  values[0] = 1;
  values[1] = 0;
  values[2] = 1;

  // Check if graph evaluation works (0.3*0.9*1*0.2*8)
  EXPECT_DOUBLES_EQUAL( 4.32, graph(values), 1e-9);

  // Below values lead to selecting the 3rd index in the ternary factor table
  values[0] = 0;
  values[1] = 1;
  values[2] = 0;

  // Check if graph evaluation works (0.9*0.6*1*0.9*4)
  EXPECT_DOUBLES_EQUAL( 1.944, graph(values), 1e-9);

  // Check if graph product works
  DecisionTreeFactor product = graph.product();
  EXPECT_DOUBLES_EQUAL( 1.944, product(values), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, test) {
  // Declare keys and ordering
  DiscreteKey C(0, 2), B(1, 2), A(2, 2);

  // A simple factor graph (A)-fAC-(C)-fBC-(B)
  // with smoothness priors
  DiscreteFactorGraph graph;
  graph.add(A & C, "3 1 1 3");
  graph.add(C & B, "3 1 1 3");

  // Test EliminateDiscrete
  const Ordering frontalKeys{0};
  const auto [conditional, newFactorPtr] = EliminateDiscrete(graph, frontalKeys);

  DecisionTreeFactor newFactor = *newFactorPtr;

  // Normalize newFactor by max for comparison with expected
  auto normalization = newFactor.max(newFactor.size());

  newFactor = newFactor / *normalization;

  // Check Conditional
  CHECK(conditional);
  Signature signature((C | B, A) = "9/1 1/1 1/1 1/9");
  DiscreteConditional expectedConditional(signature);
  EXPECT(assert_equal(expectedConditional, *conditional));

  // Check Factor
  CHECK(&newFactor);
  DecisionTreeFactor expectedFactor(B & A, "10 6 6 10");
  // Normalize by max.
  normalization = expectedFactor.max(expectedFactor.size());
  // Ensure normalization is correct.
  expectedFactor = expectedFactor / *normalization;
  EXPECT(assert_equal(expectedFactor, newFactor));

  // Test using elimination tree
  const Ordering ordering{0, 1, 2};
  DiscreteEliminationTree etree(graph, ordering);
  const auto [actual, remainingGraph] = etree.eliminate(&EliminateDiscrete);

  // Check Bayes net
  DiscreteBayesNet expectedBayesNet;
  expectedBayesNet.add(signature);
  expectedBayesNet.add(B | A = "5/3 3/5");
  expectedBayesNet.add(A % "1/1");
  EXPECT(assert_equal(expectedBayesNet, *actual));

  // Test eliminateSequential
  DiscreteBayesNet::shared_ptr actual2 = graph.eliminateSequential(ordering);
  EXPECT(assert_equal(expectedBayesNet, *actual2));

  // Test mpe
  DiscreteValues mpe { {0, 0}, {1, 0}, {2, 0}};
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(9, graph(mpe), 1e-5);  // regression

  // Test sumProduct alias with all orderings:
  auto mpeProbability = expectedBayesNet(mpe);
  EXPECT_DOUBLES_EQUAL(0.28125, mpeProbability, 1e-5);  // regression

  // Using custom ordering
  DiscreteBayesNet bayesNet = graph.sumProduct(ordering);
  EXPECT_DOUBLES_EQUAL(mpeProbability, bayesNet(mpe), 1e-5);

  for (Ordering::OrderingType orderingType :
       {Ordering::COLAMD, Ordering::METIS, Ordering::NATURAL,
        Ordering::CUSTOM}) {
    auto bayesNet = graph.sumProduct(orderingType);
    EXPECT_DOUBLES_EQUAL(mpeProbability, bayesNet(mpe), 1e-5);
  }
}

/* ************************************************************************* */
TEST_UNSAFE(DiscreteFactorGraph, testMaxProduct) {
  // Declare a bunch of keys
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);

  // Create Factor graph
  DiscreteFactorGraph graph;
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  // Created expected MPE
  DiscreteValues mpe{{0, 0}, {1, 1}, {2, 1}};

  // Do max-product with different orderings
  for (Ordering::OrderingType orderingType :
       {Ordering::COLAMD, Ordering::METIS, Ordering::NATURAL,
        Ordering::CUSTOM}) {
    DiscreteLookupDAG dag = graph.maxProduct(orderingType);
    auto actualMPE = dag.argmax();
    EXPECT(assert_equal(mpe, actualMPE));
    auto actualMPE2 = graph.optimize();  // all in one
    EXPECT(assert_equal(mpe, actualMPE2));
  }
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, marginalIsNotMPE) {
  // Declare 2 keys
  DiscreteKey A(0, 2), B(1, 2);

  // Create Bayes net such that marginal on A is bigger for 0 than 1, but the
  // MPE does not have A=0.
  DiscreteBayesNet bayesNet;
  bayesNet.add(B | A = "1/1 1/2");
  bayesNet.add(A % "10/9");

  // The expected MPE is A=1, B=1
  DiscreteValues mpe { {0, 1}, {1, 1} };

  // Which we verify using max-product:
  DiscreteFactorGraph graph(bayesNet);
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(0.315789, graph(mpe), 1e-5);  // regression
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, testMPE_Darwiche09book_p244) {
  // The factor graph in Darwiche09book, page 244
  DiscreteKey A(4, 2), C(3, 2), S(2, 2), T1(0, 2), T2(1, 2);

  // Create Factor graph
  DiscreteFactorGraph graph;
  graph.add(S, "0.55 0.45");
  graph.add(S & C, "0.05 0.95 0.01 0.99");
  graph.add(C & T1, "0.80 0.20 0.20 0.80");
  graph.add(S & C & T2, "0.80 0.20 0.20 0.80 0.95 0.05 0.05 0.95");
  graph.add(T1 & T2 & A, "1 0 0 1 0 1 1 0");
  graph.add(A, "1 0");  // evidence, A = yes (first choice in Darwiche)

  DiscreteValues mpe { {0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 0}};
  EXPECT_DOUBLES_EQUAL(0.33858, graph(mpe), 1e-5);  // regression
  // You can check visually by printing product:
  // graph.product().print("Darwiche-product");

  // Check MPE.
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));

  // Check Bayes Net
  const Ordering ordering{0, 1, 2, 3, 4};
  auto chordal = graph.eliminateSequential(ordering);
  EXPECT_LONGS_EQUAL(5, chordal->size());

  // Let us create the Bayes tree here, just for fun, because we don't use it
  DiscreteBayesTree::shared_ptr bayesTree =
      graph.eliminateMultifrontal(ordering);
  //  bayesTree->print("Bayes Tree");
  EXPECT_LONGS_EQUAL(2, bayesTree->size());
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, Dot) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  string actual = graph.dot();
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var0[label=\"0\"];\n"
      "  var1[label=\"1\"];\n"
      "  var2[label=\"2\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var0--factor0;\n"
      "  var1--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var0--factor1;\n"
      "  var2--factor1;\n"
      "}\n";
  EXPECT(actual == expected);
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, DotWithNames) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  vector<string> names{"C", "A", "B"};
  auto formatter = [names](Key key) { return names[key]; };
  string actual = graph.dot(formatter);
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var0[label=\"C\"];\n"
      "  var1[label=\"A\"];\n"
      "  var2[label=\"B\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var0--factor0;\n"
      "  var1--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var0--factor1;\n"
      "  var2--factor1;\n"
      "}\n";
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(DiscreteFactorGraph, markdown) {
  // Create Factor graph
  DiscreteFactorGraph graph;
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);
  graph.add(C & A, "0.2 0.8 0.3 0.7");
  graph.add(C & B, "0.1 0.9 0.4 0.6");

  string expected =
      "`DiscreteFactorGraph` of size 2\n"
      "\n"
      "factor 0:\n"
      "|C|A|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|0.2|\n"
      "|0|1|0.8|\n"
      "|1|0|0.3|\n"
      "|1|1|0.7|\n"
      "\n"
      "factor 1:\n"
      "|C|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|0.1|\n"
      "|0|1|0.9|\n"
      "|1|0|0.4|\n"
      "|1|1|0.6|\n\n";
  vector<string> names{"C", "A", "B"};
  auto formatter = [names](Key key) { return names[key]; };
  string actual = graph.markdown(formatter);
  EXPECT(actual == expected);

  // Make sure values are correctly displayed.
  DiscreteValues values;
  values[0] = 1;
  values[1] = 0;
  EXPECT_DOUBLES_EQUAL(0.3, graph[0]->operator()(values), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, NrAssignments) {
#ifdef GTSAM_DT_MERGING
  string expected_dfg = R"(
size: 2
factor 0:  f[ (m0,2), (m1,2), (m2,2), ]
 Choice(m2) 
 0 Choice(m1) 
 0 0 Leaf [2]    0
 0 1 Choice(m0) 
 0 1 0 Leaf [1] 0.17054468
 0 1 1 Leaf [1]    0
 1 Choice(m1) 
 1 0 Leaf [2]    0
 1 1 Choice(m0) 
 1 1 0 Leaf [1] 0.27845056
 1 1 1 Leaf [1] 0.17054468
factor 1:  f[ (m0,2), (m1,2), (m2,2), (m3,2), ]
 Choice(m3) 
 0 Choice(m2) 
 0 0 Choice(m1) 
 0 0 0 Leaf [2]    1
 0 0 1 Leaf [2] 0.015366387
 0 1 Choice(m1) 
 0 1 0 Leaf [2]    1
 0 1 1 Choice(m0) 
 0 1 1 0 Leaf [1]    1
 0 1 1 1 Leaf [1] 0.015365663
 1 Choice(m2) 
 1 0 Choice(m1) 
 1 0 0 Leaf [2]    1
 1 0 1 Choice(m0) 
 1 0 1 0 Leaf [1] 0.0094115739
 1 0 1 1 Leaf [1] 0.0094115652
 1 1 Choice(m1) 
 1 1 0 Leaf [2]    1
 1 1 1 Choice(m0) 
 1 1 1 0 Leaf [1]    1
 1 1 1 1 Leaf [1] 0.009321081
)";
#else
  string expected_dfg = R"(
size: 2
factor 0:  f[ (m0,2), (m1,2), (m2,2), ]
 Choice(m2) 
 0 Choice(m1) 
 0 0 Choice(m0) 
 0 0 0 Leaf [1]    0
 0 0 1 Leaf [1]    0
 0 1 Choice(m0) 
 0 1 0 Leaf [1] 0.27527634
 0 1 1 Leaf [1] 0.44944733
 1 Choice(m1) 
 1 0 Choice(m0) 
 1 0 0 Leaf [1]    0
 1 0 1 Leaf [1]    0
 1 1 Choice(m0) 
 1 1 0 Leaf [1]    0
 1 1 1 Leaf [1] 0.27527634
factor 1:  f[ (m0,2), (m1,2), (m2,2), (m3,2), ]
 Choice(m3) 
 0 Choice(m2) 
 0 0 Choice(m1) 
 0 0 0 Choice(m0) 
 0 0 0 0 Leaf [1]    1
 0 0 0 1 Leaf [1]    1
 0 0 1 Choice(m0) 
 0 0 1 0 Leaf [1] 0.015366387
 0 0 1 1 Leaf [1] 0.015366387
 0 1 Choice(m1) 
 0 1 0 Choice(m0) 
 0 1 0 0 Leaf [1]    1
 0 1 0 1 Leaf [1]    1
 0 1 1 Choice(m0) 
 0 1 1 0 Leaf [1]    1
 0 1 1 1 Leaf [1] 0.015365663
 1 Choice(m2) 
 1 0 Choice(m1) 
 1 0 0 Choice(m0) 
 1 0 0 0 Leaf [1]    1
 1 0 0 1 Leaf [1]    1
 1 0 1 Choice(m0) 
 1 0 1 0 Leaf [1] 0.0094115739
 1 0 1 1 Leaf [1] 0.0094115652
 1 1 Choice(m1) 
 1 1 0 Choice(m0) 
 1 1 0 0 Leaf [1]    1
 1 1 0 1 Leaf [1]    1
 1 1 1 Choice(m0) 
 1 1 1 0 Leaf [1]    1
 1 1 1 1 Leaf [1] 0.009321081
)";
#endif

  DiscreteKeys d0{{M(0), 2}, {M(1), 2}, {M(2), 2}};
  std::vector<double> p0 = {0, 0, 0.17054468, 0.27845056, 0, 0, 0, 0.17054468};
  AlgebraicDecisionTree<Key> dt(d0, p0);
  DiscreteConditional f0(3, d0, dt);

  DiscreteKeys d1{{M(0), 2}, {M(1), 2}, {M(2), 2}, {M(3), 2}};
  std::vector<double> p1 = {
      1, 1, 1, 1, 0.015366387, 0.0094115739, 1,           1,
      1, 1, 1, 1, 0.015366387, 0.0094115652, 0.015365663, 0.009321081};
  DecisionTreeFactor f1(d1, p1);

  DiscreteFactorGraph dfg;
  dfg.add(f0);
  dfg.add(f1);

  EXPECT(assert_print_equal(expected_dfg, dfg));
}

/* ************************************************************************* */
int main() {
TestResult tr;
return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

