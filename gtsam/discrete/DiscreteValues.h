/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteValues.h
 *  @date Dec 13, 2021
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/Assignment.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

/** A map from keys to values
 * TODO(dellaert): Do we need this? Should we just use gtsam::DiscreteValues?
 * We just need another special DiscreteValue to represent labels,
 * However, all other Lie's operators are undefined in this class.
 * The good thing is we can have a Hybrid graph of discrete/continuous variables
 * together..
 * Another good thing is we don't need to have the special DiscreteKey which
 * stores cardinality of a Discrete variable. It should be handled naturally in
 * the new class DiscreteValue, as the variable's type (domain)
 */
class DiscreteValues : public Assignment<Key> {
 public:
  using Assignment::Assignment; // all constructors

  // Construct from assignment.
  DiscreteValues(const Assignment<Key>& a) : Assignment<Key>(a) {}

  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << ": ";
    for (const typename Assignment::value_type& keyValue : *this)
      std::cout << "(" << keyFormatter(keyValue.first) << ", "
                << keyValue.second << ")";
    std::cout << std::endl;
  }
};

// traits
template<> struct traits<DiscreteValues> : public Testable<DiscreteValues> {};

}  // namespace gtsam
