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
 *
 */
struct DCValues {
  DCValues() {}
  DCValues(const Values &c, const DiscreteValues &d)
      : continuous(c), discrete(d) {}
  Values continuous;
  DiscreteValues discrete;
};

}  // namespace gtsam
