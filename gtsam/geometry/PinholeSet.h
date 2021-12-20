/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   PinholeSet.h
 * @brief  A CameraSet of either CalibratedCamera, PinholePose, or PinholeCamera
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/triangulation.h>
#include <boost/optional.hpp>

namespace gtsam {

/**
 * PinholeSet: triangulates point and keeps an estimate of it around.
 */
template<class CAMERA>
class PinholeSet: public CameraSet<CAMERA> {

private:
  typedef CameraSet<CAMERA> Base;
  typedef PinholeSet<CAMERA> This;

protected:

public:

  /** Virtual destructor */
  virtual ~PinholeSet() {
  }

  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const override {
    Base::print(s);
  }

  /// equals
  bool equals(const PinholeSet& p, double tol = 1e-9) const {
    return Base::equals(p, tol); // TODO all flags
  }

  /// @}

  /// triangulateSafe
  TriangulationResult triangulateSafe(
      const typename CAMERA::MeasurementVector& measured,
      const TriangulationParameters& params) const {
    return gtsam::triangulateSafe(*this, measured, params);
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

template<class CAMERA>
struct traits<PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

template<class CAMERA>
struct traits<const PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

} // \ namespace gtsam
