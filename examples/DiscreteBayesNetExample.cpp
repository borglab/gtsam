/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  DiscreteBayesNetExample.cpp
 * @brief   Discrete Bayes Net example with famous Asia Bayes Network
 * @author  Frank Dellaert
 * @date  JULY 10, 2020
 */

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/inference/BayesNet.h>

#include <iomanip>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
  DiscreteBayesNet asia;
  DiscreteKey Asia(0, 2), Smoking(4, 2), Tuberculosis(3, 2), LungCancer(6, 2),
      Bronchitis(7, 2), Either(5, 2), XRay(2, 2), Dyspnea(1, 2);
  asia.add(Asia % "99/1");
  asia.add(Smoking % "50/50");

  asia.add(Tuberculosis | Asia = "99/1 95/5");
  asia.add(LungCancer | Smoking = "99/1 90/10");
  asia.add(Bronchitis | Smoking = "70/30 40/60");

  asia.add((Either | Tuberculosis, LungCancer) = "F T T T");

  asia.add(XRay | Either = "95/5 2/98");
  asia.add((Dyspnea | Either, Bronchitis) = "9/1 2/8 3/7 1/9");

  // print
  vector<string> pretty = {"Asia",    "Dyspnea", "XRay",       "Tuberculosis",
                           "Smoking", "Either",  "LungCancer", "Bronchitis"};
  auto formatter = [pretty](Key key) { return pretty[key]; };
  asia.print("Asia", formatter);

  // Convert to factor graph
  DiscreteFactorGraph fg(asia);

  // Create solver and eliminate
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2), Key(3), Key(4), Key(5), Key(6), Key(7);

  // solve
  auto mpe = fg.optimize();
  GTSAM_PRINT(mpe);

  // We can also build a Bayes tree (directed junction tree).
  // The elimination order above will do fine:
  auto bayesTree = fg.eliminateMultifrontal(ordering);
  bayesTree->print("bayesTree", formatter);

  // add evidence, we were in Asia and we have dyspnea
  fg.add(Asia, "0 1");
  fg.add(Dyspnea, "0 1");

  // solve again, now with evidence
  auto mpe2 = fg.optimize();
  GTSAM_PRINT(mpe2);

  // We can also sample from it
  DiscreteBayesNet::shared_ptr chordal = fg.eliminateSequential(ordering);
  cout << "\n10 samples:" << endl;
  for (size_t i = 0; i < 10; i++) {
    auto sample = chordal->sample();
    GTSAM_PRINT(sample);
  }
  return 0;
}
