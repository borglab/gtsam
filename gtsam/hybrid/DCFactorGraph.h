/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DCFactorGraph.h
 * @brief  Simple class for factor graphs of DCFactor type
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */
#pragma once

#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtsam {

class DCConditional;

/**
 * Very simple class to create a factor graph with factors of type DCFactor
 */
class DCFactorGraph : public gtsam::FactorGraph<DCFactor> {
 public:
  using shared_ptr = boost::shared_ptr<DCFactorGraph>;
  using EliminationResult = std::pair<boost::shared_ptr<DCConditional>, boost::shared_ptr<DCFactor> >;
  using Eliminate = std::function<EliminationResult(const DCFactorGraph&, const Ordering&)>;

  DCFactorGraph() : FactorGraph<DCFactor>() {}
};

}  // namespace gtsam
