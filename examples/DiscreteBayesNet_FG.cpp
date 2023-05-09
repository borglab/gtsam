/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  DiscreteBayesNet_FG.cpp
 * @brief   Discrete Bayes Net example using Factor Graphs
 * @author  Abhijit
 * @date  Jun 4, 2012
 *
 * We use the famous Rain/Cloudy/Sprinkler Example of [Russell & Norvig, 2009,
 * p529] You may be familiar with other graphical model packages like BNT
 * (available at http://bnt.googlecode.com/svn/trunk/docs/usage.html) where this
 * is used as an example. The following demo is same as that in the above link,
 * except that everything is using GTSAM.
 */

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>

#include <iomanip>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
  // Define keys and a print function
  Key C(1), S(2), R(3), W(4);
  auto print = [=](DiscreteFactor::sharedValues values) {
    cout << boolalpha << "Cloudy = " << static_cast<bool>((*values)[C])
         << "  Sprinkler = " << static_cast<bool>((*values)[S])
         << "  Rain = " << boolalpha << static_cast<bool>((*values)[R])
         << "  WetGrass = " << static_cast<bool>((*values)[W]) << endl;
  };

  // We assume binary state variables
  // we have 0 == "False" and 1 == "True"
  const size_t nrStates = 2;

  // define variables
  DiscreteKey Cloudy(C, nrStates), Sprinkler(S, nrStates), Rain(R, nrStates),
      WetGrass(W, nrStates);

  // create Factor Graph of the bayes net
  DiscreteFactorGraph graph;

  // add factors
  graph.add(Cloudy, "0.5 0.5");                      // P(Cloudy)
  graph.add(Cloudy & Sprinkler, "0.5 0.5 0.9 0.1");  // P(Sprinkler | Cloudy)
  graph.add(Cloudy & Rain, "0.8 0.2 0.2 0.8");       // P(Rain | Cloudy)
  graph.add(Sprinkler & Rain & WetGrass,
            "1 0 0.1 0.9 0.1 0.9 0.001 0.99");  // P(WetGrass | Sprinkler, Rain)

  // Alternatively we can also create a DiscreteBayesNet, add
  // DiscreteConditional factors and create a FactorGraph from it. (See
  // testDiscreteBayesNet.cpp)

  // Since this is a relatively small distribution, we can as well print
  // the whole distribution..
  cout << "Distribution of Example: " << endl;
  cout << setw(11) << "Cloudy(C)" << setw(14) << "Sprinkler(S)" << setw(10)
       << "Rain(R)" << setw(14) << "WetGrass(W)" << setw(15) << "P(C,S,R,W)"
       << endl;
  for (size_t a = 0; a < nrStates; a++)
    for (size_t m = 0; m < nrStates; m++)
      for (size_t h = 0; h < nrStates; h++)
        for (size_t c = 0; c < nrStates; c++) {
          DiscreteFactor::Values values;
          values[C] = c;
          values[S] = h;
          values[R] = m;
          values[W] = a;
          double prodPot = graph(values);
          cout << setw(8) << static_cast<bool>(c) << setw(14)
               << static_cast<bool>(h) << setw(12) << static_cast<bool>(m)
               << setw(13) << static_cast<bool>(a) << setw(16) << prodPot
               << endl;
        }

  // "Most Probable Explanation", i.e., configuration with largest value
  DiscreteFactor::sharedValues mpe = graph.eliminateSequential()->optimize();
  cout << "\nMost Probable Explanation (MPE):" << endl;
  print(mpe);

  // "Inference" We show an inference query like: probability that the Sprinkler
  // was on; given that the grass is wet i.e. P( S | C=0) = ?

  // add evidence that it is not Cloudy
  graph.add(Cloudy, "1 0");

  // solve again, now with evidence
  DiscreteBayesNet::shared_ptr chordal = graph.eliminateSequential();
  DiscreteFactor::sharedValues mpe_with_evidence = chordal->optimize();

  cout << "\nMPE given C=0:" << endl;
  print(mpe_with_evidence);

  // we can also calculate arbitrary marginals:
  DiscreteMarginals marginals(graph);
  cout << "\nP(S=1|C=0):" << marginals.marginalProbabilities(Sprinkler)[1]
       << endl;
  cout << "\nP(R=0|C=0):" << marginals.marginalProbabilities(Rain)[0] << endl;
  cout << "\nP(W=1|C=0):" << marginals.marginalProbabilities(WetGrass)[1]
       << endl;

  // We can also sample from it
  cout << "\n10 samples:" << endl;
  for (size_t i = 0; i < 10; i++) {
    DiscreteFactor::sharedValues sample = chordal->sample();
    print(sample);
  }
  return 0;
}
