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

#include <exception>
#include <sstream>

namespace gtsam {

/** An inference algorithm was called with inconsistent arguments.  The factor
 * graph, ordering, or variable index were inconsistent with each other, or a
 * full elimination routine was called with an ordering that does not include
 * all of the variables. */
class InconsistentEliminationRequested : public std::exception {
  KeySet keys_;
  const KeyFormatter& keyFormatter = DefaultKeyFormatter;

 public:
  InconsistentEliminationRequested() noexcept {}

  InconsistentEliminationRequested(
      const KeySet& keys,
      const KeyFormatter& key_formatter = DefaultKeyFormatter)
      : keys_(keys), keyFormatter(key_formatter) {}

  ~InconsistentEliminationRequested() noexcept override {}
  const char* what() const noexcept override {
    // Format keys for printing
    std::stringstream sstr;
    for (auto key : keys_) {
      sstr << keyFormatter(key) << ", ";
    }
    std::string keys = sstr.str();
    // remove final comma and space.
    keys.pop_back();
    keys.pop_back();

    static std::string msg =
        "An inference algorithm was called with inconsistent "
        "arguments.  "
        "The\n"
        "factor graph, ordering, or variable index were "
        "inconsistent with "
        "each\n"
        "other, or a full elimination routine was called with "
        "an ordering "
        "that\n"
        "does not include all of the variables.\n";
    msg += ("Leftover keys after elimination: " + keys);
    return msg.c_str();
  }
};
}  // namespace gtsam
