/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    inferenceExceptions.cpp
 * @brief   Exceptions that may be thrown by inference algorithms
 * @author  Richard Roberts, Varun Agrawal
 * @date    Apr 25, 2013
 */

#include <gtsam/inference/inferenceExceptions.h>

#include <sstream>

namespace gtsam {

InconsistentEliminationRequested::InconsistentEliminationRequested(
    const KeySet& keys, const KeyFormatter& key_formatter)
    : keys_(keys.begin(), keys.end()), keyFormatter(key_formatter) {}

const char* InconsistentEliminationRequested::what() const noexcept {
  // Format keys for printing
  std::stringstream sstr;
  size_t nrKeysToDisplay = std::min(size_t(4), keys_.size());
  for (size_t i = 0; i < nrKeysToDisplay; i++) {
    sstr << keyFormatter(keys_.at(i));
    if (i < nrKeysToDisplay - 1) {
      sstr << ", ";
    }
  }
  if (keys_.size() > nrKeysToDisplay) {
    sstr << ", ... (total " << keys_.size() << " keys)";
  }
  sstr << ".";
  std::string keys = sstr.str();

  std::string msg =
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
  // `new` to allocate memory on heap instead of stack
  return (new std::string(msg))->c_str();
}
}  // namespace gtsam
