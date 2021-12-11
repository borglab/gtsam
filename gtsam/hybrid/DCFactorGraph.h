/**
 * @file DCFactorGraph.h
 * @brief Simple class for factor graphs of DCFactor type
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2020 The Ambitious Folks of the MRG
 */
#pragma once

#include <gtsam/inference/FactorGraph.h>

#include "dcsam/DCFactor.h"

namespace dcsam {

/**
 * Very simple class to create a factor graph with factors of type DCFactor
 */
class DCFactorGraph : public gtsam::FactorGraph<DCFactor> {
 public:
  DCFactorGraph() : FactorGraph<DCFactor>() {}
};

}  // namespace dcsam
