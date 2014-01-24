/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GPSFactor.h
 *  @author Frank Dellaert
 *  @brief  Header file for GPS factor
 *  @date   January 22, 2014
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/// Geodetic coordinates
class Geodetic {
private:
  double latitude_, longitude_, altitude_;
  Geodetic(double latitude, double longitude, double altitude);
public:
  Point3 toNED(const Geodetic& originNED) const {
    return Point3();
  }
};


/**
 * Prior on position in a local North-East-Down (NED) navigation frame
 * @addtogroup Navigation
 */
class GPSFactor: public NoiseModelFactor1<Pose3> {

private:

  typedef NoiseModelFactor1<Pose3> Base;

  Point3 nT_; ///< Position measurement in NED

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<GPSFactor> shared_ptr;

  /// Typedef to this class
  typedef GPSFactor This;

  /** default constructor - only use for serialization */
  GPSFactor() {
  }

  virtual ~GPSFactor() {
  }

  /**
   * Constructor from a measurement already in local NED
   * @param key of the Pose3 variable that will be constrained
   * @param gpsInNED measurement already in NED coordinates
   * @param model Gaussian noise model
   */
  GPSFactor(Key key, const Point3& gpsInNED, const SharedNoiseModel& model) :
      Base(model, key), nT_(gpsInNED) {
  }

  /**
   * Constructor that converts geodetic to local NED
   * @param key of the Pose3 variable that will be constrained
   * @param gps measurement
   * @param originNED the origin of the NED frame
   * @param model Gaussian noise model
   */
  GPSFactor(Key key, const Geodetic& gps, const Geodetic& originNED,
      const SharedNoiseModel& model) :
      Base(model, key), nT_(gps.toNED(originNED)) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "GPSFactor on " << keyFormatter(this->key()) << "\n";
    nT_.print("  prior mean: ");
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) && this->nT_.equals(e->nT_, tol);
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Pose3& p,
      boost::optional<Matrix&> H = boost::none) const {
    if (H) {
      H->resize(3, 6);
      *H << zeros(3, 3) << eye(3); // TODO make static
    }
    // manifold equivalent of h(x)-z -> log(z,h(x))
    return nT_.localCoordinates(p.translation());
  }

  const Point3 & measurementInNED() const {
    return nT_;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
};

} /// namespace gtsam
