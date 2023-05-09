/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file UGM_chain.cpp
 * @brief UGM (undirected graphical model) examples: chain
 * @author Frank Dellaert
 * @author Abhijit Kundu
 *
 * See http://www.di.ens.fr/~mschmidt/Software/UGM/chain.html
 * for more explanation. This code demos the same example using GTSAM.
 */

#include <gtsam/base/timing.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>

#include <iomanip>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // Set Number of Nodes in the Graph
  const int nrNodes = 60;

  // Each node takes 1 of 7 possible states denoted by 0-6 in following order:
  // ["VideoGames"  "Industry"  "GradSchool"  "VideoGames(with PhD)"
  // "Industry(with PhD)"  "Academia"  "Deceased"]
  const size_t nrStates = 7;

  // define variables
  vector<DiscreteKey> nodes;
  for (int i = 0; i < nrNodes; i++) {
    DiscreteKey dk(i, nrStates);
    nodes.push_back(dk);
  }

  // create graph
  DiscreteFactorGraph graph;

  // add node potentials
  graph.add(nodes[0], ".3 .6 .1 0 0 0 0");
  for (int i = 1; i < nrNodes; i++) graph.add(nodes[i], "1 1 1 1 1 1 1");

  const std::string edgePotential =
      ".08 .9 .01 0 0 0 .01 "
      ".03 .95 .01 0 0 0 .01 "
      ".06 .06 .75 .05 .05 .02 .01 "
      "0 0 0 .3 .6 .09 .01 "
      "0 0 0 .02 .95 .02 .01 "
      "0 0 0 .01 .01 .97 .01 "
      "0 0 0 0 0 0 1";

  // add edge potentials
  for (int i = 0; i < nrNodes - 1; i++)
    graph.add(nodes[i] & nodes[i + 1], edgePotential);

  cout << "Created Factor Graph with " << nrNodes << " variable nodes and "
       << graph.size() << " factors (Unary+Edge).";

  // "Decoding", i.e., configuration with largest value
  // We use sequential variable elimination
  DiscreteBayesNet::shared_ptr chordal = graph.eliminateSequential();
  DiscreteFactor::sharedValues optimalDecoding = chordal->optimize();
  optimalDecoding->print("\nMost Probable Explanation (optimalDecoding)\n");

  // "Inference" Computing marginals for each node
  // Here we'll make use of DiscreteMarginals class, which makes use of
  // bayes-tree based shortcut evaluation of marginals
  DiscreteMarginals marginals(graph);

  cout << "\nComputing Node Marginals ..(BayesTree based)" << endl;
  gttic_(Multifrontal);
  for (vector<DiscreteKey>::iterator it = nodes.begin(); it != nodes.end();
       ++it) {
    // Compute the marginal
    Vector margProbs = marginals.marginalProbabilities(*it);

    // Print the marginals
    cout << "Node#" << setw(4) << it->first << " :  ";
    print(margProbs);
  }
  gttoc_(Multifrontal);

  tictoc_print_();
  return 0;
}
