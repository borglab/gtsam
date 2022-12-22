/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
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

#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteBayesTree.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/BayesNet.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;
static constexpr bool debug = false;

/* ************************************************************************* */
struct TestFixture {
  vector<DiscreteKey> keys;
  DiscreteBayesNet bayesNet;
  boost::shared_ptr<DiscreteBayesTree> bayesTree;

  /**
   * Create a thin-tree Bayesnet, a la Jean-Guillaume Durand (former student),
   * and then create the Bayes tree from it.
   */
  TestFixture() {
    // Define variables.
    for (int i = 0; i < 15; i++) {
      DiscreteKey key_i(i, 2);
      keys.push_back(key_i);
    }

    // Create thin-tree Bayesnet.
    bayesNet.add(keys[14] % "1/3");

    bayesNet.add(keys[13] | keys[14] = "1/3 3/1");
    bayesNet.add(keys[12] | keys[14] = "3/1 3/1");

    bayesNet.add((keys[11] | keys[13], keys[14]) = "1/4 2/3 3/2 4/1");
    bayesNet.add((keys[10] | keys[13], keys[14]) = "1/4 3/2 2/3 4/1");
    bayesNet.add((keys[9] | keys[12], keys[14]) = "4/1 2/3 F 1/4");
    bayesNet.add((keys[8] | keys[12], keys[14]) = "T 1/4 3/2 4/1");

    bayesNet.add((keys[7] | keys[11], keys[13]) = "1/4 2/3 3/2 4/1");
    bayesNet.add((keys[6] | keys[11], keys[13]) = "1/4 3/2 2/3 4/1");
    bayesNet.add((keys[5] | keys[10], keys[13]) = "4/1 2/3 3/2 1/4");
    bayesNet.add((keys[4] | keys[10], keys[13]) = "2/3 1/4 3/2 4/1");

    bayesNet.add((keys[3] | keys[9], keys[12]) = "1/4 2/3 3/2 4/1");
    bayesNet.add((keys[2] | keys[9], keys[12]) = "1/4 8/2 2/3 4/1");
    bayesNet.add((keys[1] | keys[8], keys[12]) = "4/1 2/3 3/2 1/4");
    bayesNet.add((keys[0] | keys[8], keys[12]) = "2/3 1/4 3/2 4/1");

    // Create a BayesTree out of the Bayes net.
    bayesTree = DiscreteFactorGraph(bayesNet).eliminateMultifrontal();
  }
};

/* ************************************************************************* */
TEST(DiscreteBayesTree, ThinTree) {
  const TestFixture self;
  const auto& keys = self.keys;

  if (debug) {
    GTSAM_PRINT(self.bayesNet);
    self.bayesNet.saveGraph("/tmp/discreteBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  if (debug) {
    GTSAM_PRINT(*self.bayesTree);
    self.bayesTree->saveGraph("/tmp/discreteBayesTree.dot");
  }

  // Check frontals and parents
  for (size_t i : {13, 14, 9, 3, 2, 8, 1, 0, 10, 5, 4}) {
    auto clique_i = (*self.bayesTree)[i];
    EXPECT_LONGS_EQUAL(i, *(clique_i->conditional_->beginFrontals()));
  }

  auto R = self.bayesTree->roots().front();

  // Check whether BN and BT give the same answer on all configurations
  auto allPosbValues = DiscreteValues::CartesianProduct(
      keys[0] & keys[1] & keys[2] & keys[3] & keys[4] & keys[5] & keys[6] &
      keys[7] & keys[8] & keys[9] & keys[10] & keys[11] & keys[12] & keys[13] &
      keys[14]);
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteValues x = allPosbValues[i];
    double expected = self.bayesNet.evaluate(x);
    double actual = self.bayesTree->evaluate(x);
    DOUBLES_EQUAL(expected, actual, 1e-9);
  }

  // Calculate all some marginals for DiscreteValues==all1
  Vector marginals = Vector::Zero(15);
  double joint_12_14 = 0, joint_9_12_14 = 0, joint_8_12_14 = 0, joint_8_12 = 0,
         joint82 = 0, joint12 = 0, joint24 = 0, joint45 = 0, joint46 = 0,
         joint_4_11 = 0, joint_11_13 = 0, joint_11_13_14 = 0,
         joint_11_12_13_14 = 0, joint_9_11_12_13 = 0, joint_8_11_12_13 = 0;
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteValues x = allPosbValues[i];
    double px = self.bayesTree->evaluate(x);
    for (size_t i = 0; i < 15; i++)
      if (x[i]) marginals[i] += px;
    if (x[12] && x[14]) {
      joint_12_14 += px;
      if (x[9]) joint_9_12_14 += px;
      if (x[8]) joint_8_12_14 += px;
    }
    if (x[8] && x[12]) joint_8_12 += px;
    if (x[2]) {
      if (x[8]) joint82 += px;
      if (x[1]) joint12 += px;
    }
    if (x[4]) {
      if (x[2]) joint24 += px;
      if (x[5]) joint45 += px;
      if (x[6]) joint46 += px;
      if (x[11]) joint_4_11 += px;
    }
    if (x[11] && x[13]) {
      joint_11_13 += px;
      if (x[8] && x[12]) joint_8_11_12_13 += px;
      if (x[9] && x[12]) joint_9_11_12_13 += px;
      if (x[14]) {
        joint_11_13_14 += px;
        if (x[12]) {
          joint_11_12_13_14 += px;
        }
      }
    }
  }
  DiscreteValues all1 = allPosbValues.back();

  // check separator marginal P(S0)
  auto clique = (*self.bayesTree)[0];
  DiscreteFactorGraph separatorMarginal0 =
      clique->separatorMarginal(EliminateDiscrete);
  DOUBLES_EQUAL(joint_8_12, separatorMarginal0(all1), 1e-9);

  DOUBLES_EQUAL(joint_12_14, 0.1875, 1e-9);
  DOUBLES_EQUAL(joint_8_12_14, 0.0375, 1e-9);
  DOUBLES_EQUAL(joint_9_12_14, 0.15, 1e-9);

  // check separator marginal P(S9), should be P(14)
  clique = (*self.bayesTree)[9];
  DiscreteFactorGraph separatorMarginal9 =
      clique->separatorMarginal(EliminateDiscrete);
  DOUBLES_EQUAL(marginals[14], separatorMarginal9(all1), 1e-9);

  // check separator marginal of root, should be empty
  clique = (*self.bayesTree)[11];
  DiscreteFactorGraph separatorMarginal11 =
      clique->separatorMarginal(EliminateDiscrete);
  LONGS_EQUAL(0, separatorMarginal11.size());

  // check shortcut P(S9||R) to root
  clique = (*self.bayesTree)[9];
  DiscreteBayesNet shortcut = clique->shortcut(R, EliminateDiscrete);
  LONGS_EQUAL(1, shortcut.size());
  DOUBLES_EQUAL(joint_11_13_14 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S8||R) to root
  clique = (*self.bayesTree)[8];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_11_12_13_14 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S2||R) to root
  clique = (*self.bayesTree)[2];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_9_11_12_13 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S0||R) to root
  clique = (*self.bayesTree)[0];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_8_11_12_13 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // calculate all shortcuts to root
  DiscreteBayesTree::Nodes cliques = self.bayesTree->nodes();
  for (auto clique : cliques) {
    DiscreteBayesNet shortcut = clique.second->shortcut(R, EliminateDiscrete);
    if (debug) {
      clique.second->conditional_->printSignature();
      shortcut.print("shortcut:");
    }
  }

  // Check all marginals
  DiscreteFactor::shared_ptr marginalFactor;
  for (size_t i = 0; i < 15; i++) {
    marginalFactor = self.bayesTree->marginalFactor(i, EliminateDiscrete);
    double actual = (*marginalFactor)(all1);
    DOUBLES_EQUAL(marginals[i], actual, 1e-9);
  }

  DiscreteBayesNet::shared_ptr actualJoint;

  // Check joint P(8, 2)
  actualJoint = self.bayesTree->jointBayesNet(8, 2, EliminateDiscrete);
  DOUBLES_EQUAL(joint82, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(1, 2)
  actualJoint = self.bayesTree->jointBayesNet(1, 2, EliminateDiscrete);
  DOUBLES_EQUAL(joint12, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(2, 4)
  actualJoint = self.bayesTree->jointBayesNet(2, 4, EliminateDiscrete);
  DOUBLES_EQUAL(joint24, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 5)
  actualJoint = self.bayesTree->jointBayesNet(4, 5, EliminateDiscrete);
  DOUBLES_EQUAL(joint45, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 6)
  actualJoint = self.bayesTree->jointBayesNet(4, 6, EliminateDiscrete);
  DOUBLES_EQUAL(joint46, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 11)
  actualJoint = self.bayesTree->jointBayesNet(4, 11, EliminateDiscrete);
  DOUBLES_EQUAL(joint_4_11, actualJoint->evaluate(all1), 1e-9);
}

/* ************************************************************************* */
TEST(DiscreteBayesTree, Dot) {
  const TestFixture self;
  string actual = self.bayesTree->dot();
  EXPECT(actual ==
         "digraph G{\n"
         "0[label=\"13, 11, 6, 7\"];\n"
         "0->1\n"
         "1[label=\"14 : 11, 13\"];\n"
         "1->2\n"
         "2[label=\"9, 12 : 14\"];\n"
         "2->3\n"
         "3[label=\"3 : 9, 12\"];\n"
         "2->4\n"
         "4[label=\"2 : 9, 12\"];\n"
         "2->5\n"
         "5[label=\"8 : 12, 14\"];\n"
         "5->6\n"
         "6[label=\"1 : 8, 12\"];\n"
         "5->7\n"
         "7[label=\"0 : 8, 12\"];\n"
         "1->8\n"
         "8[label=\"10 : 13, 14\"];\n"
         "8->9\n"
         "9[label=\"5 : 10, 13\"];\n"
         "8->10\n"
         "10[label=\"4 : 10, 13\"];\n"
         "}");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
