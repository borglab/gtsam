/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * powerMethodExample.h
 *
 * @file   powerMethodExample.h
 * @date   Nov 2020
 * @author Jing Wu
 * @brief  Create sparse and dense factor graph for
 * PowerMethod/AcceleratedPowerMethod
 */

#include <gtsam/inference/Symbol.h>

#include <iostream>


namespace gtsam {
namespace linear {
namespace test {
namespace example {

/* ************************************************************************* */
inline GaussianFactorGraph createSparseGraph() {
  using symbol_shorthand::X;
  // Let's make a scalar synchronization graph with 4 nodes
  GaussianFactorGraph fg;
  auto model = noiseModel::Unit::Create(1);
  for (size_t j = 0; j < 3; j++) {
    fg.add(X(j), -I_1x1, X(j + 1), I_1x1, Vector1::Zero(), model);
  }
  fg.add(X(3), -I_1x1, X(0), I_1x1, Vector1::Zero(), model);  // extra row

  return fg;
}

/* ************************************************************************* */
inline GaussianFactorGraph createDenseGraph() {
  using symbol_shorthand::X;
  // Let's make a scalar synchronization graph with 10 nodes
  GaussianFactorGraph fg;
  auto model = noiseModel::Unit::Create(1);
  // Iterate over nodes
  for (size_t j = 0; j < 10; j++) {
    // Each node has an edge with all the others
    for (size_t i = 1; i < 10; i++)
    fg.add(X(j), -I_1x1, X((j + i) % 10), I_1x1, Vector1::Zero(), model);
  }

  return fg;
}

/* ************************************************************************* */

}  // namespace example
}  // namespace test
}  // namespace linear
}  // namespace gtsam
