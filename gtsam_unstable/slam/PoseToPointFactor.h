/**
 *  @file   PoseToPointFactor.hpp
 *  @brief  This factor can be used to track a 3D landmark over time by providing
 *          local measurements of its location.
 *  @author David Wisth
 **/
#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * A class for a measurement between a pose and a point.
 * @addtogroup SLAM
 */
class PoseToPointFactor: public NoiseModelFactor2<Pose3, Point3> {

private:

  typedef PoseToPointFactor This;
  typedef NoiseModelFactor2<Pose3, Point3> Base;

  Point3 measured_; /** the point measurement */

public:

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<PoseToPointFactor> shared_ptr;

  /** default constructor - only use for serialization */
  PoseToPointFactor() {}

  /** Constructor */
  PoseToPointFactor(Key key1, Key key2, const Point3& measured,
                    const SharedNoiseModel& model) :
    Base(model, key1, key2), measured_(measured)
  {
  }

  virtual ~PoseToPointFactor() {}

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "PoseToPointFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n"
        << "  measured: " << measured_.transpose() << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol) && traits<Point3>::Equals(this->measured_, e->measured_, tol);
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors
    * Error = pose_est.inverse()*point_est - measured_
    * (The error is in the measurement coordinate system)
    */
  Vector evaluateError(const Pose3& W_T_WI,
                       const Point3& W_r_WC,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const
  {
    return W_T_WI.transformTo(W_r_WC, H1, H2) - measured_;
  }

  /** return the measured */
  const Point3& measured() const {
    return measured_;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }

}; // \class PoseToPointFactor

} /// namespace gtsam
