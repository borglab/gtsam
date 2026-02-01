/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    inferenceExceptions.h
 * @brief   Exceptions that may be thrown by inference algorithms
 * @author  Richard Roberts, Varun Agrawal
 * @date    Apr 25, 2013
 */
#pragma once

#include <gtsam/global_includes.h>
#include <gtsam/inference/Key.h>

#include <exception>

namespace gtsam {

/** An inference algorithm was called with inconsistent arguments.  The factor
 * graph, ordering, or variable index were inconsistent with each other, or a
 * full elimination routine was called with an ordering that does not include
 * all of the variables. */
class InconsistentEliminationRequested : public std::exception {
  KeyVector keys_;
  const KeyFormatter& keyFormatter = DefaultKeyFormatter;

 public:
  InconsistentEliminationRequested() noexcept {}

  InconsistentEliminationRequested(
      const KeySet& keys,
      const KeyFormatter& key_formatter = DefaultKeyFormatter);

  ~InconsistentEliminationRequested() noexcept override {}

  const char* what() const noexcept override;
};
}  // namespace gtsam
