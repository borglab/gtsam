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

#include <vector>

using namespace std;
using namespace gtsam;

static bool debug = false;

/* ************************************************************************* */

TEST_UNSAFE(DiscreteBayesTree, ThinTree) {
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
  bayesNet.add(key[14] % "1/3");

  bayesNet.add(key[13] | key[14] = "1/3 3/1");
  bayesNet.add(key[12] | key[14] = "3/1 3/1");

  bayesNet.add((key[11] | key[13], key[14]) = "1/4 2/3 3/2 4/1");
  bayesNet.add((key[10] | key[13], key[14]) = "1/4 3/2 2/3 4/1");
  bayesNet.add((key[9] | key[12], key[14]) = "4/1 2/3 F 1/4");
  bayesNet.add((key[8] | key[12], key[14]) = "T 1/4 3/2 4/1");

  bayesNet.add((key[7] | key[11], key[13]) = "1/4 2/3 3/2 4/1");
  bayesNet.add((key[6] | key[11], key[13]) = "1/4 3/2 2/3 4/1");
  bayesNet.add((key[5] | key[10], key[13]) = "4/1 2/3 3/2 1/4");
  bayesNet.add((key[4] | key[10], key[13]) = "2/3 1/4 3/2 4/1");

  bayesNet.add((key[3] | key[9], key[12]) = "1/4 2/3 3/2 4/1");
  bayesNet.add((key[2] | key[9], key[12]) = "1/4 8/2 2/3 4/1");
  bayesNet.add((key[1] | key[8], key[12]) = "4/1 2/3 3/2 1/4");
  bayesNet.add((key[0] | key[8], key[12]) = "2/3 1/4 3/2 4/1");

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/discreteBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  auto bayesTree = DiscreteFactorGraph(bayesNet).eliminateMultifrontal();
  if (debug) {
    GTSAM_PRINT(*bayesTree);
    bayesTree->saveGraph("/tmp/discreteBayesTree.dot");
  }

  // Check frontals and parents
  for (size_t i : {13, 14, 9, 3, 2, 8, 1, 0, 10, 5, 4}) {
    auto clique_i = (*bayesTree)[i];
    EXPECT_LONGS_EQUAL(i, *(clique_i->conditional_->beginFrontals()));
  }

  auto R = bayesTree->roots().front();

  // Check whether BN and BT give the same answer on all configurations
  vector<DiscreteFactor::Values> allPosbValues = cartesianProduct(
      key[0] & key[1] & key[2] & key[3] & key[4] & key[5] & key[6] & key[7] &
      key[8] & key[9] & key[10] & key[11] & key[12] & key[13] & key[14]);
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteFactor::Values x = allPosbValues[i];
    double expected = bayesNet.evaluate(x);
    double actual = bayesTree->evaluate(x);
    DOUBLES_EQUAL(expected, actual, 1e-9);
  }

  // Calculate all some marginals for Values==all1
  Vector marginals = Vector::Zero(15);
  double joint_12_14 = 0, joint_9_12_14 = 0, joint_8_12_14 = 0, joint_8_12 = 0,
         joint82 = 0, joint12 = 0, joint24 = 0, joint45 = 0, joint46 = 0,
         joint_4_11 = 0, joint_11_13 = 0, joint_11_13_14 = 0,
         joint_11_12_13_14 = 0, joint_9_11_12_13 = 0, joint_8_11_12_13 = 0;
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteFactor::Values x = allPosbValues[i];
    double px = bayesTree->evaluate(x);
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
  DiscreteFactor::Values all1 = allPosbValues.back();

  // check separator marginal P(S0)
  auto clique = (*bayesTree)[0];
  DiscreteFactorGraph separatorMarginal0 =
      clique->separatorMarginal(EliminateDiscrete);
  DOUBLES_EQUAL(joint_8_12, separatorMarginal0(all1), 1e-9);

  // check separator marginal P(S9), should be P(14)
  clique = (*bayesTree)[9];
  DiscreteFactorGraph separatorMarginal9 =
      clique->separatorMarginal(EliminateDiscrete);
  DOUBLES_EQUAL(marginals[14], separatorMarginal9(all1), 1e-9);

  // check separator marginal of root, should be empty
  clique = (*bayesTree)[11];
  DiscreteFactorGraph separatorMarginal11 =
      clique->separatorMarginal(EliminateDiscrete);
  LONGS_EQUAL(0, separatorMarginal11.size());

  // check shortcut P(S9||R) to root
  clique = (*bayesTree)[9];
  DiscreteBayesNet shortcut = clique->shortcut(R, EliminateDiscrete);
  LONGS_EQUAL(1, shortcut.size());
  DOUBLES_EQUAL(joint_11_13_14 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S8||R) to root
  clique = (*bayesTree)[8];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_11_12_13_14 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S2||R) to root
  clique = (*bayesTree)[2];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_9_11_12_13 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // check shortcut P(S0||R) to root
  clique = (*bayesTree)[0];
  shortcut = clique->shortcut(R, EliminateDiscrete);
  DOUBLES_EQUAL(joint_8_11_12_13 / joint_11_13, shortcut.evaluate(all1), 1e-9);

  // calculate all shortcuts to root
  DiscreteBayesTree::Nodes cliques = bayesTree->nodes();
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
    marginalFactor = bayesTree->marginalFactor(i, EliminateDiscrete);
    double actual = (*marginalFactor)(all1);
    DOUBLES_EQUAL(marginals[i], actual, 1e-9);
  }

  DiscreteBayesNet::shared_ptr actualJoint;

  // Check joint P(8, 2)
  actualJoint = bayesTree->jointBayesNet(8, 2, EliminateDiscrete);
  DOUBLES_EQUAL(joint82, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(1, 2)
  actualJoint = bayesTree->jointBayesNet(1, 2, EliminateDiscrete);
  DOUBLES_EQUAL(joint12, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(2, 4)
  actualJoint = bayesTree->jointBayesNet(2, 4, EliminateDiscrete);
  DOUBLES_EQUAL(joint24, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 5)
  actualJoint = bayesTree->jointBayesNet(4, 5, EliminateDiscrete);
  DOUBLES_EQUAL(joint45, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 6)
  actualJoint = bayesTree->jointBayesNet(4, 6, EliminateDiscrete);
  DOUBLES_EQUAL(joint46, actualJoint->evaluate(all1), 1e-9);

  // Check joint P(4, 11)
  actualJoint = bayesTree->jointBayesNet(4, 11, EliminateDiscrete);
  DOUBLES_EQUAL(joint_4_11, actualJoint->evaluate(all1), 1e-9);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
