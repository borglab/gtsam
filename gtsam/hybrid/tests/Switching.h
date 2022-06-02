/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file Switching.h
 *  @date Mar 11, 2022
 *  @author Varun Agrawal
 *  @author Fan Jiang
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#pragma once

using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::X;

namespace gtsam {
inline HybridGaussianFactorGraph::shared_ptr makeSwitchingChain(
    size_t n, std::function<Key(int)> keyFunc = X,
    std::function<Key(int)> dKeyFunc = C) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(keyFunc(1), I_3x3, Z_3x1));

  // keyFunc(1) to keyFunc(n+1)
  for (size_t t = 1; t < n; t++) {
    hfg.add(GaussianMixtureFactor::FromFactors(
        {keyFunc(t), keyFunc(t + 1)}, {{dKeyFunc(t), 2}},
        {boost::make_shared<JacobianFactor>(keyFunc(t), I_3x3, keyFunc(t + 1),
                                            I_3x3, Z_3x1),
         boost::make_shared<JacobianFactor>(keyFunc(t), I_3x3, keyFunc(t + 1),
                                            I_3x3, Vector3::Ones())}));

    if (t > 1) {
      hfg.add(DecisionTreeFactor({{dKeyFunc(t - 1), 2}, {dKeyFunc(t), 2}},
                                 "0 1 1 3"));
    }
  }

  return boost::make_shared<HybridGaussianFactorGraph>(std::move(hfg));
}

inline std::pair<KeyVector, std::vector<int>> makeBinaryOrdering(
    std::vector<Key> &input) {
  KeyVector new_order;
  std::vector<int> levels(input.size());
  std::function<void(std::vector<Key>::iterator, std::vector<Key>::iterator,
                     int)>
      bsg = [&bsg, &new_order, &levels, &input](
                std::vector<Key>::iterator begin,
                std::vector<Key>::iterator end, int lvl) {
        if (std::distance(begin, end) > 1) {
          std::vector<Key>::iterator pivot =
              begin + std::distance(begin, end) / 2;

          new_order.push_back(*pivot);
          levels[std::distance(input.begin(), pivot)] = lvl;
          bsg(begin, pivot, lvl + 1);
          bsg(pivot + 1, end, lvl + 1);
        } else if (std::distance(begin, end) == 1) {
          new_order.push_back(*begin);
          levels[std::distance(input.begin(), begin)] = lvl;
        }
      };

  bsg(input.begin(), input.end(), 0);
  std::reverse(new_order.begin(), new_order.end());
  // std::reverse(levels.begin(), levels.end());
  return {new_order, levels};
}

}  // namespace gtsam
