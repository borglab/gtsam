/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Value.h>

#include <cstddef>
#include <string>

#if defined(_WIN32)
#ifdef VALUES_CROSS_LIBRARY_EXPORTS
#define VALUES_CROSS_LIBRARY_EXPORT __declspec(dllexport)
#else
#define VALUES_CROSS_LIBRARY_EXPORT __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define VALUES_CROSS_LIBRARY_EXPORT __attribute__((visibility("default")))
#else
#define VALUES_CROSS_LIBRARY_EXPORT
#endif

namespace values_cross_library {

/** A small manifold whose RTTI crosses the regression-test library boundary. */
class VALUES_CROSS_LIBRARY_EXPORT TestValue {
 public:
  static constexpr size_t dimension = 1;

  /// Constructs the test value.
  explicit TestValue(double value = 0.0) : value_(value) {}

  /// Prints the scalar value.
  void print(const std::string& label = "") const;
  /// Compares two values within a tolerance.
  bool equals(const TestValue& other, double tolerance = 1e-9) const;
  /// Returns the tangent-space dimension.
  size_t dim() const;
  /// Retracts by the supplied tangent vector.
  TestValue retract(const gtsam::Vector& delta) const;
  /// Computes local coordinates to another value.
  gtsam::Vector localCoordinates(const TestValue& other) const;

  /// Returns the wrapped scalar.
  double value() const { return value_; }

 private:
  double value_;
};

/** Creates a Value in the helper shared library. */
VALUES_CROSS_LIBRARY_EXPORT gtsam::Value* createValue(double value);

}  // namespace values_cross_library

namespace gtsam {

template <>
struct traits<values_cross_library::TestValue>
    : public internal::Manifold<values_cross_library::TestValue> {};

}  // namespace gtsam
