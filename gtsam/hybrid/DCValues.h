/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *
 * @file   DCSAM_types.h
 * @brief  Values container for DCSAM
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/nonlinear/Values.h>

#include <utility>

namespace gtsam {

/**
 * @brief Container for both discrete and continuous values.
 */
struct DCValues {
  DCValues() {}

  DCValues(const Values &continuous, const DiscreteValues &discrete)
      : continuous_(continuous), discrete_(discrete) {}

  Values continuous_;
  DiscreteValues discrete_;
};

}  // namespace gtsam
