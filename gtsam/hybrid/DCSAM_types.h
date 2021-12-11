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

namespace dcsam {

using DiscreteValues = gtsam::DiscreteFactor::Values;

struct DCValues {
  DCValues() {}
  DCValues(const gtsam::Values &c, const DiscreteValues &d)
      : continuous(c), discrete(d) {}
  gtsam::Values continuous;
  DiscreteValues discrete;
};

struct DCMarginals {
  DCMarginals(const gtsam::Marginals &c, const gtsam::DiscreteMarginals &d)
      : continuous(c), discrete(d) {}
  gtsam::Marginals continuous;
  gtsam::DiscreteMarginals discrete;
};

}  // namespace dcsam
