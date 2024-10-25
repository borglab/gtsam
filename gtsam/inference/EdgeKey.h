/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file EdgeKey.h
 * @date Oct 24, 2024
 * @author: Frank Dellaert
 * @author: Akshay Krishnan
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>

namespace gtsam {
class GTSAM_EXPORT EdgeKey {
 protected:
  std::uint32_t i_;  ///< Upper 32 bits
  std::uint32_t j_;  ///< Lower 32 bits

 public:
  /// @name Constructors
  /// @{

  /// Default constructor
  EdgeKey() : i_(0), j_(0) {}

  /// Constructor
  EdgeKey(std::uint32_t i, std::uint32_t j) : i_(i), j_(j) {}

  EdgeKey(Key key)
      : i_(static_cast<std::uint32_t>(key >> 32)),
        j_(static_cast<std::uint32_t>(key)) {}

  /// @}
  /// @name API
  /// @{

  /// Cast to Key
  operator Key() const { return ((std::uint64_t)i_ << 32) | j_; }

  /// Retrieve high 32 bits
  inline std::uint32_t i() const { return i_; }

  /// Retrieve low 32 bits
  inline std::uint32_t j() const { return j_; }

  /** Create a string from the key */
  operator std::string() const;

  /// Output stream operator
  friend GTSAM_EXPORT std::ostream& operator<<(std::ostream&, const EdgeKey&);

  /// @}
  /// @name Testable
  /// @{

  /// Prints the EdgeKey with an optional prefix string.
  void print(const std::string& s = "") const;

  /// Checks if this EdgeKey is equal to another, tolerance is ignored.
  bool equals(const EdgeKey& expected, double tol = 0.0) const {
    return (*this) == expected;
  }
  /// @}
};

/// traits
template <>
struct traits<EdgeKey> : public Testable<EdgeKey> {};

}  // namespace gtsam
