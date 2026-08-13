/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file AsVectorSpace.h
 * @brief Explicit vector-space adapter for manifold-valued classes.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/base/VectorSpace.h>

namespace gtsam {

/**
 * Opt-in adapter that equips a manifold class with additive vector-space
 * operations.
 *
 * The default-constructed wrapped value is chosen as the additive identity.
 * Addition and subtraction use the wrapped manifold's Local and Retract maps.
 * This is useful when an application deliberately chooses a local affine
 * embedding for a type, such as a camera calibration, that has no natural
 * group composition.
 *
 * @tparam Class A type satisfying GTSAM's manifold traits.
 */
template <class Class>
class AsVectorSpace : public Class {
 public:
  using TangentVector = typename traits<Class>::TangentVector;
  inline constexpr static int dimension = traits<Class>::dimension;

  /// Construct the selected additive identity.
  AsVectorSpace() : Class(Class()) {}

  /// Wrap an existing manifold value.
  explicit AsVectorSpace(const Class& value) : Class(value) {}

  /// Retract a tangent vector from the selected identity.
  explicit AsVectorSpace(const TangentVector& tangent)
      : Class(traits<Class>::Retract(Class(), tangent)) {}

  /// Return the selected additive identity.
  static AsVectorSpace Identity() { return AsVectorSpace(); }

  AsVectorSpace operator+(const AsVectorSpace& other) const {
    const TangentVector tangent =
        traits<Class>::Local(Class(), static_cast<const Class&>(other));
    return AsVectorSpace(
        traits<Class>::Retract(static_cast<const Class&>(*this), tangent));
  }

  AsVectorSpace operator-(const AsVectorSpace& other) const {
    const TangentVector tangent =
        traits<Class>::Local(Class(), static_cast<const Class&>(other));
    return AsVectorSpace(
        traits<Class>::Retract(static_cast<const Class&>(*this), -tangent));
  }

  AsVectorSpace operator-() const {
    const TangentVector tangent =
        traits<Class>::Local(Class(), static_cast<const Class&>(*this));
    return AsVectorSpace(traits<Class>::Retract(Class(), -tangent));
  }

  AsVectorSpace operator+(const TangentVector& tangent) const {
    return AsVectorSpace(
        traits<Class>::Retract(static_cast<const Class&>(*this), tangent));
  }

  AsVectorSpace operator-(const TangentVector& tangent) const {
    return AsVectorSpace(
        traits<Class>::Retract(static_cast<const Class&>(*this), -tangent));
  }

  /// Return coordinates relative to the selected additive identity.
  TangentVector vector() const {
    return traits<Class>::Local(Class(), static_cast<const Class&>(*this));
  }
};

template <class Class>
struct traits<AsVectorSpace<Class>>
    : internal::VectorSpaceTraits<AsVectorSpace<Class>>,
      Testable<AsVectorSpace<Class>> {};

template <class Class>
struct traits<const AsVectorSpace<Class>> : traits<AsVectorSpace<Class>> {};

}  // namespace gtsam
