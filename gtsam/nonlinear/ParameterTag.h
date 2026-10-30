/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/dllexport.h>

#include <cstddef>
#include <memory>
#include <string>

namespace gtsam {

/**
 * Identifies a parameter vector for factor VJP queries. Construction creates a
 * new identity; copies share that identity and keep it alive. The optional name
 * is only a display label: two independently constructed tags with the same
 * name are different parameters. A tag does not own parameter values or record
 * factor dependencies. Reuse the same tag across factors and training steps.
 * Tags identify parameters within a running process and are not serialized.
 */
class GTSAM_EXPORT ParameterTag {
 public:
  /** Create a distinct identity with an optional display name. */
  explicit ParameterTag(std::string name = "");

  /** Copy a tag while retaining its shared identity. */
  ParameterTag(const ParameterTag& other) = default;

  /** Assign another tag's shared identity. */
  ParameterTag& operator=(const ParameterTag& other) = default;

  /// Return the display name; names do not determine equality.
  const std::string& name() const { return identity_->name; }

  /// Compare shared identities.
  bool equals(const ParameterTag& other) const {
    return identity_ == other.identity_;
  }

  /// Compare shared identities.
  bool operator==(const ParameterTag& other) const { return equals(other); }

  /// Compare shared identities.
  bool operator!=(const ParameterTag& other) const { return !equals(other); }

  /// Hash the shared identity for associative containers in wrapped languages.
  size_t hash() const { return std::hash<decltype(identity_)>{}(identity_); }

  /** Print the display name. */
  void print(const std::string& prefix = "ParameterTag: ") const;

 private:
  struct Identity {
    std::string name;
  };
  std::shared_ptr<const Identity> identity_;
};

}  // namespace gtsam
