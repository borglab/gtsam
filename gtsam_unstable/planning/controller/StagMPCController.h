/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file StagMPCController.h
 * @author Akash Ratheesh
 * @author Frank Dellaert
 */
#pragma once

#include <gtsam/planning/MPCController.h>

namespace gtsam {

/// Common MPC controller interface used by STAG planners.
using StagMPCController = MPCController;

/// Shared parameters of STAG planners.
using StagMPCControllerParams = MPCControllerParams;

}  // namespace gtsam
