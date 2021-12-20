/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *
 * @file   DCMarginals.h
 * @brief  Marginals container for DCSAM
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/nonlinear/Marginals.h>

#include <utility>

namespace gtsam {

struct DCMarginals {
  DCMarginals(const Marginals &c, const DiscreteMarginals &d)
      : continuous(c), discrete(d) {}
  Marginals continuous;
  DiscreteMarginals discrete;
};

}  // namespace gtsam
