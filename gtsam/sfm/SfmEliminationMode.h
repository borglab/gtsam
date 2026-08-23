/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmEliminationMode.h
 * @brief Shared CPU/CUDA elimination policy for SFM optimizers.
 */

#pragma once

namespace gtsam {

/// Determines whether an SFM optimizer solves the joint or reduced system.
enum class SfmEliminationMode {
  Full,   ///< Send the complete system to the selected solver.
  Schur,  ///< Eliminate landmark variables before solving the reduced system.
};

}  // namespace gtsam
