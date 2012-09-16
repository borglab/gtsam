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

  /**
   * Determine variable indices to keep in recursive separator shortcut calculation
   * The factor graph p_Cp_B has keys from the parent clique Cp and from B.
   * But we only keep the variables not in S union B.
   */
  vector<Index> indices(derived_ptr B,
      const FactorGraph<FactorType>& p_Cp_B) const {

    // Get all keys
    set<Index> allKeys = p_Cp_B.keys();

    // We do this by first merging S and B
    boost::iterator_range<FactorType::iterator> indicesS =
        this->conditional()->parents();
    size_t sizeS = indicesS.end() - indicesS.begin();
    vector<Index> &indicesB = B->conditional()->keys();
    vector<Index> S_union_B(indicesB.size() + sizeS);
    vector<Index>::iterator it = set_union(indicesS.begin(), indicesS.end(),
        indicesB.begin(), indicesB.end(), S_union_B.begin());

    // then intersecting S_union_B with allKeys
    vector<Index> keepers(indicesB.size() + sizeS);
    it = set_intersection(S_union_B.begin(), it, allKeys.begin(), allKeys.end(),
        keepers.begin());
    keepers.erase(it, keepers.end());

    return keepers;
  }

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

  /**
   * Separator shortcut function P(S||B) = P(S\B|B)
   * where S is a clique separator, and B any node (e.g., a brancing in the tree)
   * We can compute it recursively from the parent shortcut
   * P(Sp||B) as \int P(Fp|Sp) P(Sp||B), where Fp are the frontal nodes in p
   */
  FactorGraph<FactorType>::shared_ptr separatorShortcut(derived_ptr B) const {

    FactorGraph<FactorType>::shared_ptr p_S_B; //shortcut P(S||B) This is empty now

    // We only calculate the shortcut when this clique is not B
    derived_ptr parent(parent_.lock());
    if (B.get() != this) {

      // Obtain P(Fp|Sp) as a factor
      boost::shared_ptr<FactorType> p_Fp_Sp = parent->conditional()->toFactor();

      // Obtain the parent shortcut P(Sp|B) as factors
      // TODO: really annoying that we eliminate more than we have to !
      // TODO: we should only eliminate C_p\B, with S\B variables last
      // TODO: and this index dance will be easier then, as well
      FactorGraph<FactorType> p_Sp_B(parent->shortcut(B, &EliminateDiscrete));

      // now combine P(Cp||B) = P(Fp|Sp) * P(Sp||B)
      FactorGraph<FactorType> p_Cp_B;
      p_Cp_B.push_back(p_Fp_Sp);
      p_Cp_B.push_back(p_Sp_B);

      // Create a generic solver that will marginalize for us
      GenericSequentialSolver<FactorType> solver(p_Cp_B);

      // The factor graph above will have keys from the parent clique Cp and from B.
      // But we only keep the variables not in S union B.
      vector<Index> keepers = indices(B, p_Cp_B);

      p_S_B = solver.jointFactorGraph(keepers, &EliminateDiscrete);
    }
    // return the shortcut P(S||B)
    return p_S_B;
  }

  /**
   * The shortcut density is a conditional P(S||B) of the separator of this
   * clique on the clique B.
   */
  BayesNet<DiscreteConditional> shortcut(derived_ptr B,
      Eliminate function) const {

    //Check if the ShortCut already exists
    if (cachedShortcut_) {
      return *cachedShortcut_; // return the cached version
    } else {
      BayesNet<DiscreteConditional> bn;
      FactorGraph<FactorType>::shared_ptr fg = separatorShortcut(B);
      if (fg) {
        // calculate set S\B of indices to keep in Bayes net
        vector<Index> indicesS(this->conditional()->beginParents(),
            this->conditional()->endParents());
        // now get B indices out
        vector<Index> &indicesB = B->conditional()->keys();
        vector<Index> S_setminus_B(indicesS.size());
        vector<Index>::iterator it = set_difference(indicesS.begin(),
            indicesS.end(), indicesB.begin(), indicesB.end(),
            S_setminus_B.begin());
        S_setminus_B.erase(it, S_setminus_B.end());
        set<Index> keep(S_setminus_B.begin(), S_setminus_B.end());
        BOOST_FOREACH (FactorType::shared_ptr factor,*fg) {
          DecisionTreeFactor::shared_ptr df = boost::dynamic_pointer_cast<
              DecisionTreeFactor>(factor);
          if (keep.count(*factor->begin()))
            bn.push_front(boost::make_shared<DiscreteConditional>(1, *df));
        }
      }
      cachedShortcut_ = bn;
      return bn;
    }
  }

};

typedef BayesTree<DiscreteConditional, Clique> DiscreteBayesTree;

/* ************************************************************************* */
double evaluate(const DiscreteBayesTree& tree,
    const DiscreteConditional::Values & values) {
  return tree.root()->evaluate(values);
}

/* ************************************************************************* */

TEST_UNSAFE( DiscreteMarginals, thinTree ) {

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
  double shortcut0, sum0;
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
    // calculate a deep shortcut
    if (x[12] && x[14] & x[8])
      shortcut0 += actual;
    if (x[14])
      sum0 += actual;
  }
  DiscreteFactor::Values all1 = allPosbValues.back();

  // check shortcut P(S0||R) to root
  Clique::shared_ptr R = bayesTree.root();
  Clique::shared_ptr c = bayesTree[0];
  DiscreteBayesNet shortcut = c->shortcut(R, &EliminateDiscrete);
  EXPECT_DOUBLES_EQUAL(shortcut0/sum0, evaluate(shortcut,all1), 1e-9);

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

