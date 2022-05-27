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

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/inference/BayesNet.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

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
  DiscreteValues mpe;
  insert(mpe)(0, 2)(1, 1)(2, 0)(3, 0);
  EXPECT(assert_equal(mpe, actualMPE));
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, debugSchedulerTable) {
  DiscreteKey PC(0, 4), ME(1, 4), AI(2, 4), A(3, 3);

  DiscreteFactorGraph graph;
  graph.push_back(TableFactor(AI, "1 0 0 1"));
  graph.push_back(TableFactor(AI, "1 1 1 0"));
  graph.push_back(TableFactor(A & AI, "1 1 1 0   1 1 1 1   0 1 1 1"));
  graph.push_back(TableFactor(ME, "0 1 0 0"));
  graph.push_back(TableFactor(ME, "1 1 1 0"));
  graph.push_back(TableFactor(A & ME, "1 1 1 0   1 1 1 1   0 1 1 1"));
  graph.push_back(TableFactor(PC, "1 0 1 0"));
  graph.push_back(TableFactor(PC, "1 1 1 0"));
  graph.push_back(TableFactor(A & PC, "1 1 1 0   1 1 1 1   0 1 1 1"));
  graph.push_back(TableFactor(ME & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0"));
  graph.push_back(TableFactor(PC & ME, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0"));
  graph.push_back(TableFactor(PC & AI, "0 1 1 1   1 0 1 1   1 1 0 1  1 1 1 0"));

  // Check MPE.
  auto actualMPE = graph.optimize();
  DiscreteValues mpe;
  insert(mpe)(0, 2)(1, 1)(2, 0)(3, 0);
  EXPECT(assert_equal(mpe, actualMPE));
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
/// Test the () operator of DiscreteFactorGraph
TEST(DiscreteFactorGraph, DiscreteFactorGraphEvaluationTestTable) {
  // Three keys P1 and P2
  DiscreteKey P1(0, 2), P2(1, 2), P3(2, 3);

  // Create the DiscreteFactorGraph
  DiscreteFactorGraph graph;
  TableFactor f1(P1, "0.9 0.3");
  TableFactor f2(P2, "0.9 0.6");
  TableFactor f3(P1 & P2, "4 1 10 4");
  graph.push_back(f1);
  graph.push_back(f2);
  graph.push_back(f3);

  // Instantiate DiscreteValues
  DiscreteValues values;
  values[0] = 1;
  values[1] = 1;

  // Check if graph evaluation works ( 0.3*0.6*4 )
  EXPECT_DOUBLES_EQUAL(.72, graph(values), 1e-9);

  // Creating a new test with third node and adding unary and ternary factors on
  // it
  TableFactor f4(P3, "0.9 0.2 0.5");
  TableFactor f5(P1 & P2 & P3, "1 2 3 4 5 6 7 8 9 10 11 12");
  graph.push_back(f4);
  graph.push_back(f5);

  // Below values lead to selecting the 8th index in the ternary factor table
  values[0] = 1;
  values[1] = 0;
  values[2] = 1;

  // Check if graph evaluation works (0.3*0.9*1*0.2*8)
  EXPECT_DOUBLES_EQUAL(4.32, graph(values), 1e-9);

  // Below values lead to selecting the 3rd index in the ternary factor table
  values[0] = 0;
  values[1] = 1;
  values[2] = 0;

  // Check if graph evaluation works (0.9*0.6*1*0.9*4)
  EXPECT_DOUBLES_EQUAL(1.944, graph(values), 1e-9);

  // Check if graph product works
  DecisionTreeFactor product = graph.product();
  EXPECT_DOUBLES_EQUAL(1.944, product(values), 1e-9);
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
  Ordering frontalKeys;
  frontalKeys += Key(0);
  DiscreteConditional::shared_ptr conditional;
  DecisionTreeFactor::shared_ptr newFactor;
  boost::tie(conditional, newFactor) = EliminateDiscrete(graph, frontalKeys);

  // Check Conditional
  CHECK(conditional);
  Signature signature((C | B, A) = "9/1 1/1 1/1 1/9");
  DiscreteConditional expectedConditional(signature);
  EXPECT(assert_equal(expectedConditional, *conditional));

  // Check Factor
  CHECK(newFactor);
  DecisionTreeFactor expectedFactor(B & A, "10 6 6 10");
  EXPECT(assert_equal(expectedFactor, *newFactor));

  // Test using elimination tree
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2);
  DiscreteEliminationTree etree(graph, ordering);
  DiscreteBayesNet::shared_ptr actual;
  DiscreteFactorGraph::shared_ptr remainingGraph;
  boost::tie(actual, remainingGraph) = etree.eliminate(&EliminateDiscrete);

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
  DiscreteValues mpe;
  insert(mpe)(0, 0)(1, 0)(2, 0);
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

/* **************************************************************************/
TEST(DiscreteFactorGraph, testTable) {
  // Declare keys and ordering
  DiscreteKey C(0, 2), B(1, 2), A(2, 2);

  // A simple factor graph (A)-fAC-(C)-fBC-(B)
  // with smoothness priors
  DiscreteFactorGraph graph;
  graph.push_back(TableFactor(A & C, "3 1 1 3"));
  graph.push_back(TableFactor(C & B, "3 1 1 3"));

  // Test EliminateDiscrete
  Ordering frontalKeys;
  frontalKeys += Key(0);
  DiscreteConditional::shared_ptr conditional;
  DecisionTreeFactor::shared_ptr newFactor;
  boost::tie(conditional, newFactor) = EliminateDiscrete(graph, frontalKeys);

  // Check Conditional
  CHECK(conditional);
  Signature signature((C | B, A) = "9/1 1/1 1/1 1/9");
  DiscreteConditional expectedConditional(signature);
  EXPECT(assert_equal(expectedConditional, *conditional));

  // Check Factor
  CHECK(newFactor);
  DecisionTreeFactor expectedFactor(B & A, "10 6 6 10");
  EXPECT(assert_equal(expectedFactor, *newFactor));

  // Test using elimination tree
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2);
  DiscreteEliminationTree etree(graph, ordering);
  DiscreteBayesNet::shared_ptr actual;
  DiscreteFactorGraph::shared_ptr remainingGraph;
  boost::tie(actual, remainingGraph) = etree.eliminate(&EliminateDiscrete);

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
  DiscreteValues mpe;
  insert(mpe)(0, 0)(1, 0)(2, 0);
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
  DiscreteValues mpe;
  insert(mpe)(0, 0)(1, 1)(2, 1);

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

/* **************************************************************************/ 
TEST_UNSAFE(DiscreteFactorGraph, testMaxProductTable) {
  // Declare a bunch of keys
  DiscreteKey C(0, 2), A(1, 2), B(2, 2);

  // Create Factor graph
  DiscreteFactorGraph graph;
  graph.push_back(TableFactor(C & A, "0.2 0.8 0.3 0.7"));
  graph.push_back(TableFactor(C & B, "0.1 0.9 0.4 0.6"));

  // Created expected MPE
  DiscreteValues mpe;
  insert(mpe)(0, 0)(1, 1)(2, 1);

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
  DiscreteValues mpe;
  insert(mpe)(0, 1)(1, 1);

  // Which we verify using max-product:
  DiscreteFactorGraph graph(bayesNet);
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));
  EXPECT_DOUBLES_EQUAL(0.315789, graph(mpe), 1e-5);  // regression

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  // Optimize on BayesNet maximizes marginal, then the conditional marginals:
  auto notOptimal = bayesNet.optimize();
  EXPECT(graph(notOptimal) < graph(mpe));
  EXPECT_DOUBLES_EQUAL(0.263158, graph(notOptimal), 1e-5);  // regression
#endif
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

  DiscreteValues mpe;
  insert(mpe)(4, 0)(2, 1)(3, 1)(0, 1)(1, 1);
  EXPECT_DOUBLES_EQUAL(0.33858, graph(mpe), 1e-5);  // regression
  // You can check visually by printing product:
  // graph.product().print("Darwiche-product");

  // Check MPE.
  auto actualMPE = graph.optimize();
  EXPECT(assert_equal(mpe, actualMPE));

  // Check Bayes Net
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2), Key(3), Key(4);
  auto chordal = graph.eliminateSequential(ordering);
  EXPECT_LONGS_EQUAL(5, chordal->size());
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  auto notOptimal = chordal->optimize();  // not MPE !
  EXPECT(graph(notOptimal) < graph(mpe));
#endif

  // Let us create the Bayes tree here, just for fun, because we don't use it
  DiscreteBayesTree::shared_ptr bayesTree =
      graph.eliminateMultifrontal(ordering);
  //  bayesTree->print("Bayes Tree");
  EXPECT_LONGS_EQUAL(2, bayesTree->size());
}

#ifdef OLD

/* ************************************************************************* */
/**
 * Key type for discrete conditionals
 * Includes name and cardinality
 */
class Key2 {
private:
std::string wff_;
size_t cardinality_;
public:
/** Constructor, defaults to binary */
Key2(const std::string& name, size_t cardinality = 2) :
wff_(name), cardinality_(cardinality) {
}
const std::string& name() const {
  return wff_;
}

/** provide streaming */
friend std::ostream& operator <<(std::ostream &os, const Key2 &key);
};

struct Factor2 {
std::string wff_;
Factor2() :
wff_("@") {
}
Factor2(const std::string& s) :
wff_(s) {
}
Factor2(const Key2& key) :
wff_(key.name()) {
}

friend std::ostream& operator <<(std::ostream &os, const Factor2 &f);
friend Factor2 operator -(const Key2& key);
};

std::ostream& operator <<(std::ostream &os, const Factor2 &f) {
os << f.wff_;
return os;
}

/** negation */
Factor2 operator -(const Key2& key) {
return Factor2("-" + key.name());
}

/** OR */
Factor2 operator ||(const Factor2 &factor1, const Factor2 &factor2) {
return Factor2(std::string("(") + factor1.wff_ + " || " + factor2.wff_ + ")");
}

/** AND */
Factor2 operator &&(const Factor2 &factor1, const Factor2 &factor2) {
return Factor2(std::string("(") + factor1.wff_ + " && " + factor2.wff_ + ")");
}

/** implies */
Factor2 operator >>(const Factor2 &factor1, const Factor2 &factor2) {
return Factor2(std::string("(") + factor1.wff_ + " >> " + factor2.wff_ + ")");
}

struct Graph2: public std::list<Factor2> {

/** Add a factor graph*/
//  void operator +=(const Graph2& graph) {
//    for(const Factor2& f: graph)
//        push_back(f);
//  }
friend std::ostream& operator <<(std::ostream &os, const Graph2& graph);

};

/** Add a factor */
//Graph2 operator +=(Graph2& graph, const Factor2& factor) {
//  graph.push_back(factor);
//  return graph;
//}
std::ostream& operator <<(std::ostream &os, const Graph2& graph) {
for(const Factor2& f: graph)
os << f << endl;
return os;
}

/* ************************************************************************* */
TEST(DiscreteFactorGraph, Sugar)
{
Key2 M("Mythical"), I("Immortal"), A("Mammal"), H("Horned"), G("Magical");

// Test this desired construction
Graph2 unicorns;
unicorns += M >> -A;
unicorns += (-M) >> (-I && A);
unicorns += (I || A) >> H;
unicorns += H >> G;

// should be done by adapting boost::assign:
// unicorns += (-M) >> (-I && A), (I || A) >> H , H >> G;

cout << unicorns;
}
#endif

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
int main() {
TestResult tr;
return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

