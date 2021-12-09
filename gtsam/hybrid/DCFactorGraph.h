/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DCFactorGraph.h
 * @date December 2021
 * @brief Simple class for factor graphs of HybridFactor type
 * @author Kevin Doherty, Varun Agrawal
 */
#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtsam {

/**
 * Very simple class to create a factor graph with factors of type DCFactor
 */
class DCFactorGraph : public gtsam::FactorGraph<HybridFactor> {
 public:
  DCFactorGraph() : FactorGraph<HybridFactor>() {}
};

}  // namespace gtsam