/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file DistanceFactor.h
 * @author Duy-Nguyen Ta
 * @date Sep 26, 2014
 * 
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to constrain known measured distance between two points
 */
template<class POINT>
class DistanceFactor: public NoiseModelFactor2<POINT, POINT> {

  double measured_; /// measured distance

  typedef NoiseModelFactor2<POINT, POINT> Base;
  typedef DistanceFactor<POINT> This;

public:
  /// Default constructor
  DistanceFactor() {
  }

  /// Constructor with keys and known measured distance
  DistanceFactor(Key p1, Key p2, double measured, const SharedNoiseModel& model) :
    Base(model, p1, p2), measured_(measured) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// h(x)-z
  Vector evaluateError(const POINT& p1, const POINT& p2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    double distance = p1.distance(p2, H1, H2);
    return (Vector(1) << distance - measured_);
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && fabs(this->measured_ - e->measured_) < tol;
  }

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "DistanceFactor, distance = " << measured_ << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};

} /* namespace gtsam */
