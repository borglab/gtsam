/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file UGM_small.cpp
 * @brief UGM (undirected graphical model) examples: small
 * @author Frank Dellaert
 *
 * See http://www.di.ens.fr/~mschmidt/Software/UGM/small.html
 */

#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // We will assume 2-state variables, where, to conform to the "small" example
  // we have 0 == "right answer" and 1 == "wrong answer"
  size_t nrStates = 2;

  // define variables
  DiscreteKey Cathy(1, nrStates), Heather(2, nrStates), Mark(3, nrStates),
      Allison(4, nrStates);

  // create graph
  DiscreteFactorGraph graph;

  // add node potentials
  graph.add(Cathy,   "1 3");
  graph.add(Heather, "9 1");
  graph.add(Mark,    "1 3");
  graph.add(Allison, "9 1");

  // add edge potentials
  graph.add(Cathy & Heather, "2 1 1 2");
  graph.add(Heather & Mark,  "2 1 1 2");
  graph.add(Mark & Allison,  "2 1 1 2");

  // Print the UGM distribution
  cout << "\nUGM distribution:" << endl;
  vector<DiscreteFactor::Values> allPosbValues = cartesianProduct(
      Cathy & Heather & Mark & Allison);
  for (size_t i = 0; i < allPosbValues.size(); ++i) {
    DiscreteFactor::Values values = allPosbValues[i];
    double prodPot = graph(values);
    cout << values[Cathy.first] << " " << values[Heather.first] << " "
        << values[Mark.first] << " " << values[Allison.first] << " :\t"
        << prodPot << "\t" << prodPot / 3790 << endl;
  }

  // "Decoding", i.e., configuration with largest value (MPE)
  // We use sequential variable elimination
  DiscreteBayesNet::shared_ptr chordal = graph.eliminateSequential();
  DiscreteFactor::sharedValues optimalDecoding = chordal->optimize();
  optimalDecoding->print("\noptimalDecoding");

  // "Inference" Computing marginals
  cout << "\nComputing Node Marginals .." << endl;
  DiscreteMarginals marginals(graph);

  Vector margProbs = marginals.marginalProbabilities(Cathy);
  print(margProbs, "Cathy's Node Marginal:");

  margProbs = marginals.marginalProbabilities(Heather);
  print(margProbs, "Heather's Node Marginal");

  margProbs = marginals.marginalProbabilities(Mark);
  print(margProbs, "Mark's Node Marginal");

  margProbs = marginals.marginalProbabilities(Allison);
  print(margProbs, "Allison's Node Marginal");

  return 0;
}

