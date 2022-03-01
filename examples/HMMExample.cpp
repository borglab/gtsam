/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  DiscreteBayesNetExample.cpp
 * @brief   Hidden Markov Model example, discrete.
 * @author  Frank Dellaert
 * @date  July 12, 2020
 */

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/BayesNet.h>

#include <iomanip>
#include <sstream>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
  const int nrNodes = 4;
  const size_t nrStates = 3;

  // Define variables as well as ordering
  Ordering ordering;
  vector<DiscreteKey> keys;
  for (int k = 0; k < nrNodes; k++) {
    DiscreteKey key_i(k, nrStates);
    keys.push_back(key_i);
    ordering.emplace_back(k);
  }

  // Create HMM as a DiscreteBayesNet
  DiscreteBayesNet hmm;

  // Define backbone
  const string transition = "8/1/1 1/8/1 1/1/8";
  for (int k = 1; k < nrNodes; k++) {
    hmm.add(keys[k] | keys[k - 1] = transition);
  }

  // Add some measurements, not needed for all time steps!
  hmm.add(keys[0] % "7/2/1");
  hmm.add(keys[1] % "1/9/0");
  hmm.add(keys.back() % "5/4/1");

  // print
  hmm.print("HMM");

  // Convert to factor graph
  DiscreteFactorGraph factorGraph(hmm);

  // Do max-prodcut
  auto mpe = factorGraph.optimize();
  GTSAM_PRINT(mpe);

  // Create solver and eliminate
  // This will create a DAG ordered with arrow of time reversed
  DiscreteBayesNet::shared_ptr chordal =
      factorGraph.eliminateSequential(ordering);
  chordal->print("Eliminated");

  // We can also sample from it
  cout << "\n10 samples:" << endl;
  for (size_t k = 0; k < 10; k++) {
    auto sample = chordal->sample();
    GTSAM_PRINT(sample);
  }

  // Or compute the marginals. This re-eliminates the FG into a Bayes tree
  cout << "\nComputing Node Marginals .." << endl;
  DiscreteMarginals marginals(factorGraph);
  for (int k = 0; k < nrNodes; k++) {
    Vector margProbs = marginals.marginalProbabilities(keys[k]);
    stringstream ss;
    ss << "marginal " << k;
    print(margProbs, ss.str());
  }

  // TODO(frank): put in the glue to have DiscreteMarginals produce *arbitrary*
  // joints efficiently, by the Bayes tree shortcut magic. All the code is there
  // but it's not yet connected.

  return 0;
}
