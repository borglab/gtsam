/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalParameters.h
 * @brief Parameters for the imperative multifrontal solver.
 * @author Frank Dellaert
 * @date   January 2026
 */

#pragma once

#include <cstddef>
#include <iosfwd>

namespace gtsam {

/**
 * Parameters for `gtsam::MultifrontalSolver`.
 *
 * These parameters are intentionally defined in a standalone header so they
 * can be referenced from nonlinear optimizer parameter types without pulling
 * in the full multifrontal solver headers.
 *
 * @note LM-style damping behavior is controlled by `NonlinearMultifrontalSolver`
 * and its damping parameters; `MultifrontalParameters` only controls structure,
 * traversal, and reporting.
 */
struct MultifrontalParameters {
  enum class QRMode { Off, Allow, Force };
  size_t leafMergeDimCap = 256;           ///< Leaf-merge cap (0 disables).
  size_t mergeDimCap = 32;                ///< Merge threshold (0 disables).
  QRMode qrMode = QRMode::Allow;          ///< QR mode for leaf cliques.
  double qrAspectRatio = 2.0;             ///< Aspect ratio for QR mode=Allow.
  std::ostream* reportStream = nullptr;   ///< Optional structure reporting.
  int eliminationParallelThreshold = 10;  ///< Post-order task threshold.
  int solutionParallelThreshold = 4096;   ///< Pre-order task threshold.
  size_t numThreads = 0;  ///< Worker count (0 uses 0.75 * hw threads).
};

}  // namespace gtsam
