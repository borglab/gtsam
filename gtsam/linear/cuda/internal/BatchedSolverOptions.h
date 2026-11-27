/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedSolverOptions.h
 * @brief   Device-selection options shared by every batched CUDA backend
 * @author  Shaji
 * @date    Nov 24, 2026
 */

#pragma once

namespace gtsam::cuda {

/**
 * Options for a batched CUDA backend. deviceId of -1 keeps whatever
 * device is current for the calling thread; >= 0 pins this instance to
 * one device. Spanning GPUs means one instance per device, combined by the caller.
 */
struct BatchedSolverOptions {
  int deviceId = -1;
};

}  // namespace gtsam::cuda
