/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testDiscreteBayesTree.cpp
 * @date sept 15, 2012
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/BayesTree.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static bool debug = false;

/**
 * Custom clique class to debug shortcuts
 */
class Clique: public BayesTreeCliqueBase<Clique, DiscreteConditional> {

protected:

public:

  typedef BayesTreeCliqueBase<Clique, DiscreteConditional> Base;
  typedef boost::shared_ptr<Clique> shared_ptr;

  // Constructors
  Clique() {
  }
  Clique(const DiscreteConditional::shared_ptr& conditional) :
      Base(conditional) {
  }
  Clique(
      const std::pair<DiscreteConditional::shared_ptr,
          DiscreteConditional::FactorType::shared_ptr>& result) :
      Base(result) {
  }

  /// print index signature only
  void printSignature(const std::string& s = "Clique: ",
      const IndexFormatter& indexFormatter = DefaultIndexFormatter) const {
    ((IndexConditional::shared_ptr) conditional_)->print(s, indexFormatter);
  }

  /// evaluate value of sub-tree
  double evaluate(const DiscreteConditional::Values & values) {
    double result = (*(this->conditional_))(values);
    // evaluate all children and multiply into result
    BOOST_FOREACH(boost::shared_ptr<Clique> c, children_)
      result *= c->evaluate(values);
    return result;
  }

};

typedef BayesTree<DiscreteConditional, Clique> DiscreteBayesTree;

/* ************************************************************************* */
double evaluate(const DiscreteBayesTree& tree,
    const DiscreteConditional::Values & values) {
  return tree.root()->evaluate(values);
}

/* ************************************************************************* */

TEST_UNSAFE( DiscreteBayesTree, thinTree ) {

  const int nrNodes = 15;
  const size_t nrStates = 2;

// define variables
  vector<DiscreteKey> key;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey key_i(i, nrStates);
    key.push_back(key_i);
  }

// create a thin-tree Bayesnet, a la Jean-Guillaume
  DiscreteBayesNet bayesNet;
  add_front(bayesNet, key[14] % "1/3");

  add_front(bayesNet, key[13] | key[14] = "1/3 3/1");
  add_front(bayesNet, key[12] | key[14] = "3/1 3/1");

  add_front(bayesNet, (key[11] | key[13], key[14]) = "1/4 2/3 3/2 4/1");
  add_front(bayesNet, (key[10] | key[13], key[14]) = "1/4 3/2 2/3 4/1");
  add_front(bayesNet, (key[9] | key[12], key[14]) = "4/1 2/3 F 1/4");
  add_front(bayesNet, (key[8] | key[12], key[14]) = "T 1/4 3/2 4/1");

  add_front(bayesNet, (key[7] | key[11], key[13]) = "1/4 2/3 3/2 4/1");
  add_front(bayesNet, (key[6] | key[11], key[13]) = "1/4 3/2 2/3 4/1");
  add_front(bayesNet, (key[5] | key[10], key[13]) = "4/1 2/3 3/2 1/4");
  add_front(bayesNet, (key[4] | key[10], key[13]) = "2/3 1/4 3/2 4/1");

  add_front(bayesNet, (key[3] | key[9], key[12]) = "1/4 2/3 3/2 4/1");
  add_front(bayesNet, (key[2] | key[9], key[12]) = "1/4 8/2 2/3 4/1");
  add_front(bayesNet, (key[1] | key[8], key[12]) = "4/1 2/3 3/2 1/4");
  add_front(bayesNet, (key[0] | key[8], key[12]) = "2/3 1/4 3/2 4/1");

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/discreteBayesNet.dot");
  }

// create a BayesTree out of a Bayes net
  DiscreteBayesTree bayesTree(bayesNet);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/discreteBayesTree.dot");
  }

// Check whether BN and BT give the same answer on all configurations
// Also calculate all some marginals
  Vector marginals = zero(15);
  double shortcut8, shortcut0;
  vector<DiscreteFactor::Values> allPosbValues = cartesianProduct(
      key[0] & key[1] & key[2] & key[3] & key[4] & key[5] & key[6] & key[7]
          & key[8] & key[9] & key[10] & key[11] & key[12] & key[13] & key[14]);
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteFactor::Values x = allPosbValues[i];
    double expected = evaluate(bayesNet, x);
    double actual = evaluate(bayesTree, x);
    DOUBLES_EQUAL(expected, actual, 1e-9);
    // collect marginals
    for (size_t i = 0; i < 15; i++)
      if (x[i])
        marginals[i] += actual;
    // calculate shortcut 8 and 0
    if (x[12] && x[14])
      shortcut8 += actual;
    if (x[8] && x[12] & x[14])
      shortcut0 += actual;
  }
  DiscreteFactor::Values all1 = allPosbValues.back();

// check shortcut P(S9||R) to root
  Clique::shared_ptr R = bayesTree.root();
  Clique::shared_ptr c = bayesTree[9];
  DiscreteBayesNet shortcut = c->shortcut(R, &EliminateDiscrete);
  EXPECT_LONGS_EQUAL(0, shortcut.size());

// check shortcut P(S8||R) to root
  c = bayesTree[8];
  shortcut = c->shortcut(R, &EliminateDiscrete);
  EXPECT_DOUBLES_EQUAL(shortcut8/marginals[14], evaluate(shortcut,all1), 1e-9);

// check shortcut P(S0||R) to root
  c = bayesTree[0];
  shortcut = c->shortcut(R, &EliminateDiscrete);
  EXPECT_DOUBLES_EQUAL(shortcut0/marginals[14], evaluate(shortcut,all1), 1e-9);

// calculate all shortcuts to root
  DiscreteBayesTree::Nodes cliques = bayesTree.nodes();
  BOOST_FOREACH(Clique::shared_ptr c, cliques) {
    DiscreteBayesNet shortcut = c->shortcut(R, &EliminateDiscrete);
    if (debug) {
      c->printSignature();
      shortcut.print("shortcut:");
    }
  }

// Check all marginals
  DiscreteFactor::shared_ptr marginalFactor;
  for (size_t i = 0; i < 15; i++) {
    marginalFactor = bayesTree.marginalFactor(i, &EliminateDiscrete);
    DiscreteFactor::Values x;
    x[i] = 1;
    double actual = (*marginalFactor)(x);
    EXPECT_DOUBLES_EQUAL(marginals[i], actual, 1e-9);
  }

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

