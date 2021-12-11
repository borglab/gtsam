/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *
 * @file   DCSAM_types.h
 * @brief  Some convenient types for DCSAM
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/nonlinear/Marginals.h>

#include <utility>

namespace gtsam {

using DiscreteValues = DiscreteFactor::Values;

struct DCValues {
  DCValues() {}
  DCValues(const Values &c, const DiscreteValues &d)
      : continuous(c), discrete(d) {}
  Values continuous;
  DiscreteValues discrete;
};

struct DCMarginals {
  DCMarginals(const Marginals &c, const DiscreteMarginals &d)
      : continuous(c), discrete(d) {}
  Marginals continuous;
  DiscreteMarginals discrete;
};

}  // namespace gtsam
