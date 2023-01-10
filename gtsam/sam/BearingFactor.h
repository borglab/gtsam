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

#include <gtsam/nonlinear/ExpressionFactor.h>

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
struct BearingFactor : public ExpressionFactorN<T, A1, A2> {
  typedef ExpressionFactorN<T, A1, A2> Base;

  /// default constructor
  BearingFactor() {}

  /// primary constructor
  BearingFactor(Key key1, Key key2, const T& measured,
                const SharedNoiseModel& model)
      : Base({key1, key2}, model, measured) {
    this->initialize(expression({key1, key2}));
  }

  // Return measurement expression
  Expression<T> expression(const typename Base::ArrayNKeys &keys) const override {
    Expression<A1> a1_(keys[0]);
    Expression<A2> a2_(keys[1]);
    return Expression<T>(Bearing<A1, A2>(), a1_, a2_);
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const override {
    std::cout << s << "BearingFactor" << std::endl;
    Base::print(s, kf);
  }
  
  Vector evaluateError(const A1& a1, const A2& a2,
    OptionalMatrixType H1 = OptionalNone, OptionalMatrixType H2 = OptionalNone) const {
    std::vector<Matrix> Hs(2);
    const auto &keys = Factor::keys();
    const Vector error = unwhitenedError(
      {{keys[0], genericValue(a1)}, {keys[1], genericValue(a2)}}, 
      Hs);
    if (H1) *H1 = Hs[0];
    if (H2) *H2 = Hs[1];
    return error;
  }


 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
};  // BearingFactor

/// traits
template <typename A1, typename A2, typename T>
struct traits<BearingFactor<A1, A2, T> >
    : public Testable<BearingFactor<A1, A2, T> > {};

}  // namespace gtsam
