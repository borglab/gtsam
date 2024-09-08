/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file TranslationFactor.h
 * @author Frank Dellaert
 * @date March 2020
 * @brief Binary factor for a relative translation direction measurement.
 */

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor for a relative translation direction measurement
 * w_aZb. The measurement equation is
 *    w_aZb = Unit3(Tb - Ta)
 * i.e., w_aZb is the translation direction from frame A to B, in world
 * coordinates. Although Unit3 instances live on a manifold, following
 * Wilson14eccv_1DSfM.pdf error we compute the *chordal distance* in the
 * ambient world coordinate frame:
 *    normalized(Tb - Ta) - w_aZb.point3()
 *
 * @ingroup sfm
 */
class TranslationFactor : public NoiseModelFactorN<Point3, Point3> {
 private:
  typedef NoiseModelFactorN<Point3, Point3> Base;
  Point3 measured_w_aZb_;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor2<Point3, Point3>::evaluateError;

  /// default constructor
  TranslationFactor() {}

  TranslationFactor(Key a, Key b, const Unit3& w_aZb,
                    const SharedNoiseModel& noiseModel)
      : Base(noiseModel, a, b), measured_w_aZb_(w_aZb.point3()) {}

  /**
   * @brief Caclulate error: (norm(Tb - Ta) - measurement)
   * where Tb and Ta are Point3 translations and measurement is
   * the Unit3 translation direction from a to b.
   *
   * @param Ta translation for key a
   * @param Tb translation for key b
   * @param H1 optional jacobian in Ta
   * @param H2 optional jacobian in Tb
   * @return * Vector
   */
  Vector evaluateError(const Point3& Ta, const Point3& Tb,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    const Point3 dir = Tb - Ta;
    Matrix33 H_predicted_dir;
    const Point3 predicted =
        normalize(dir, H1 || H2 ? &H_predicted_dir : nullptr);
    if (H1) *H1 = -H_predicted_dir;
    if (H2) *H2 = H_predicted_dir;
    return predicted - measured_w_aZb_;
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \ TranslationFactor

/**
 * Binary factor for a relative translation direction measurement
 * w_aZb. The measurement equation is
 *    w_aZb = scale * (Tb - Ta)
 * i.e., w_aZb is the translation direction from frame A to B, in world
 * coordinates.
 *
 * Instead of normalizing the Tb - Ta vector, we multiply it by a scalar
 * which is also jointly optimized. This is inspired by the work on
 * BATA (Zhuang et al, CVPR 2018).
 *
 * @ingroup sfm
 */
class BilinearAngleTranslationFactor
    : public NoiseModelFactorN<Point3, Point3, Vector1> {
 private:
  typedef NoiseModelFactorN<Point3, Point3, Vector1> Base;
  Point3 measured_w_aZb_;

 public:
  /// default constructor
  BilinearAngleTranslationFactor() {}

  BilinearAngleTranslationFactor(Key a, Key b, Key scale_key,
                                 const Unit3& w_aZb,
                                 const SharedNoiseModel& noiseModel)
      : Base(noiseModel, a, b, scale_key), measured_w_aZb_(w_aZb.point3()) {}

  /**
   * @brief Caclulate error: (scale * (Tb - Ta) - measurement)
   * where Tb and Ta are Point3 translations and measurement is
   * the Unit3 translation direction from a to b.
   *
   * @param Ta translation for key a
   * @param Tb translation for key b
   * @param H1 optional jacobian in Ta
   * @param H2 optional jacobian in Tb
   * @return * Vector
   */
  Vector evaluateError(const Point3& Ta, const Point3& Tb, const Vector1& scale,
                       OptionalMatrixType H1, OptionalMatrixType H2,
                       OptionalMatrixType H3) const override {
    // Ideally we should use a positive real valued scalar datatype for scale.
    const double abs_scale = abs(scale[0]);
    const Point3 predicted = (Tb - Ta) * abs_scale;
    if (H1) {
      *H1 = -Matrix3::Identity() * abs_scale;
    }
    if (H2) {
      *H2 = Matrix3::Identity() * abs_scale;
    }
    if (H3) {
      *H3 = scale[0] >= 0 ? (Tb - Ta) : (Ta - Tb);
    }
    return predicted - measured_w_aZb_;
  }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Base", boost::serialization::base_object<Base>(*this));
  }
};  // \ BilinearAngleTranslationFactor
}  // namespace gtsam
