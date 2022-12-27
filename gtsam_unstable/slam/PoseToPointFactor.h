/**
 *  @file   PoseToPointFactor.h
 *  @brief  This factor can be used to model relative position measurements
 *  from a (2D or 3D) pose to a landmark
 *  @author David Wisth
 *  @author Luca Carlone
 **/
#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <ostream>

namespace gtsam {

/**
 * A class for a measurement between a pose and a point.
 * @ingroup slam
 */
template<typename POSE = Pose3, typename POINT = Point3>
class PoseToPointFactor : public NoiseModelFactorN<POSE, POINT> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(PoseToPointFactor, 2);

 private:
  typedef PoseToPointFactor This;
  typedef NoiseModelFactorN<POSE, POINT> Base;

  POINT measured_; /** the point measurement in local coordinates */

 public:
  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<PoseToPointFactor> shared_ptr;

  /** default constructor - only use for serialization */
  PoseToPointFactor() {}

  /** Constructor */
  PoseToPointFactor(Key key1, Key key2, const POINT& measured,
                    const SharedNoiseModel& model)
      : Base(model, key1, key2), measured_(measured) {}

  virtual ~PoseToPointFactor() {}

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "PoseToPointFactor("
              << keyFormatter(this->template key<1>()) << ","
              << keyFormatter(this->template key<2>()) << ")\n"
              << "  measured: " << measured_.transpose() << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<POINT>::Equals(this->measured_, e->measured_, tol);
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors
   * @brief Error = w_T_b.inverse()*w_P - measured_
   * @param w_T_b The pose of the body in world coordinates
   * @param w_P The estimated point location in world coordinates
   *
   * Note: measured_ and the error are in local coordiantes.
   */
  Vector evaluateError(
      const POSE& w_T_b, const POINT& w_P,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const override {
    return w_T_b.transformTo(w_P, H1, H2) - measured_;
  }

  /** return the measured */
  const POINT& measured() const { return measured_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }

};  // \class PoseToPointFactor

}  // namespace gtsam
