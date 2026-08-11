/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * AsiaExample.h
 *
 *  @date Jan, 2025
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {
namespace asia_example {

[[maybe_unused]] static const Key D = Symbol('D', 1), X = Symbol('X', 2),
                                  E = Symbol('E', 3), B = Symbol('B', 4),
                                  L = Symbol('L', 5), T = Symbol('T', 6),
                                  S = Symbol('S', 7), A = Symbol('A', 8);

[[maybe_unused]] static const DiscreteKey Dyspnea(D, 2), XRay(X, 2),
    Either(E, 2), Bronchitis(B, 2), LungCancer(L, 2), Tuberculosis(T, 2),
    Smoking(S, 2), Asia(A, 2);

// Function to construct the Asia priors
[[maybe_unused]] static DiscreteBayesNet createPriors() {
  DiscreteBayesNet priors;
  priors.add(Smoking % "50/50");
  priors.add(Asia, "99/1");
  return priors;
}

// Function to construct the incomplete Asia example
[[maybe_unused]] static DiscreteBayesNet createFragment() {
  DiscreteBayesNet fragment;
  fragment.add((Either | Tuberculosis, LungCancer) = "F T T T");
  fragment.add(LungCancer | Smoking = "99/1 90/10");
  fragment.add(Tuberculosis | Asia = "99/1 95/5");
  for (const auto& factor : createPriors()) fragment.push_back(factor);
  return fragment;
}

// Function to construct the Asia example
[[maybe_unused]] static DiscreteBayesNet createAsiaExample() {
  DiscreteBayesNet asia;
  asia.add((Dyspnea | Either, Bronchitis) = "9/1 2/8 3/7 1/9");
  asia.add(XRay | Either = "95/5 2/98");
  asia.add(Bronchitis | Smoking = "70/30 40/60");
  for (const auto& factor : createFragment()) asia.push_back(factor);
  return asia;
}
}  // namespace asia_example
}  // namespace gtsam
