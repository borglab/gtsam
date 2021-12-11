/**
 *
 * @file DCSAM_types.h
 * @brief Some convenient types for DCSAM
 * @author Kevin Doherty, kdoherty@mit.edu
 * Copyright 2021 The Ambitious Folks of the MRG
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
