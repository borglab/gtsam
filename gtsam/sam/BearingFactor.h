/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.h
 *  @brief Serializable factor induced by a bearing measurement
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

// forward declaration of Bearing functor, assumed partially specified
template <typename A1, typename A2>
struct Bearing;

/**
 * Binary factor for a bearing measurement
 * Works for any two types A1,A2 for which the functor Bearing<A1,A2>() is
 * defined
 * @ingroup sam
 */
template <typename A1, typename A2,
          typename T = typename Bearing<A1, A2>::result_type>
struct BearingFactor : public NoiseModelFactorN<A1, A2> {
  typedef BearingFactor<A1, A2, T> This;
  typedef NoiseModelFactorN<A1, A2> Base;

  T measured_;

  /// default constructor
  BearingFactor() = default;

  /// Construct from keys, a bearing measurement, and a noise model.
  BearingFactor(Key key1, Key key2, const T& measured,
                const SharedNoiseModel& model)
      : Base(model, key1, key2), measured_(measured) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// @return the measurement
  const T& measured() const { return measured_; }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const override {
    std::cout << s << "BearingFactor" << std::endl;
    Base::print(s, kf);
    traits<T>::Print(measured_, "  measured: ");
  }

  /// Check equality up to a tolerance.
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const This* p = dynamic_cast<const This*>(&f);
    return p != nullptr && Base::equals(f, tol) &&
           traits<T>::Equals(measured_, p->measured_, tol);
  }

  /// Evaluate the unwhitened bearing error and optional Jacobians.
  Vector evaluateError(const A1& a1, const A2& a2,
                       OptionalMatrixType H1 = OptionalNone,
                       OptionalMatrixType H2 = OptionalNone) const override {
    const T predicted = Bearing<A1, A2>()(a1, a2, H1, H2);
    return -traits<T>::Local(predicted, measured_);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  /// Serialize this factor.
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }
#endif
};  // BearingFactor

/// traits
template <typename A1, typename A2, typename T>
struct traits<BearingFactor<A1, A2, T> >
    : public Testable<BearingFactor<A1, A2, T> > {};

}  // namespace gtsam
