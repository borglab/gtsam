/**
 * @file RelativeElevationFactor.h
 *
 * @brief Factor representing a known relative altitude in global frame
 *
 * @date Aug 17, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Binary factor for a relative elevation.  Note that this
 * factor takes into account only elevation, and corrects for orientation.
 * Unlike a range factor, the relative elevation is signed, and only affects
 * the Z coordinate.  Measurement function h(pose, pt) = h.z() - pt.z()
 *
 * Dimension: 1
 *
 * TODO: enable use of a Pose3 for the target as well
 */
class GTSAM_UNSTABLE_EXPORT RelativeElevationFactor: public NoiseModelFactor2<Pose3, Point3> {
private:

  double measured_; /** measurement */

  typedef RelativeElevationFactor This;
  typedef NoiseModelFactor2<Pose3, Point3> Base;

public:

  RelativeElevationFactor() : measured_(0.0) {} /* Default constructor */

  RelativeElevationFactor(Key poseKey, Key pointKey, double measured,
      const SharedNoiseModel& model);

  ~RelativeElevationFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose, const Point3& point,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override;

  /** return the measured */
  inline double measured() const { return measured_; }

  /** equals specialized to this factor */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override;

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
}; // RelativeElevationFactor


} // \namespace gtsam


