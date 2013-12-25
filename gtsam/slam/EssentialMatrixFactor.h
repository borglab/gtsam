/*
 * @file EssentialMatrixFactor.cpp
 * @brief EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/base/LieScalar.h>
#include <iostream>

namespace gtsam {

/**
 * Factor that evaluates epipolar error p'Ep for given essential matrix
 */
class EssentialMatrixFactor: public NoiseModelFactor1<EssentialMatrix> {

  Point2 pA_, pB_; ///< Measurements in image A and B
  Vector vA_, vB_; ///< Homogeneous versions

  typedef NoiseModelFactor1<EssentialMatrix> Base;
  typedef EssentialMatrixFactor This;

public:

  /// Constructor
  EssentialMatrixFactor(Key key, const Point2& pA, const Point2& pB,
      const SharedNoiseModel& model) :
      Base(model, key), pA_(pA), pB_(pB), //
      vA_(EssentialMatrix::Homogeneous(pA)), //
      vB_(EssentialMatrix::Homogeneous(pB)) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast < gtsam::NonlinearFactor
        > (gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor with measurements\n  ("
        << pA_.vector().transpose() << ")' and (" << pB_.vector().transpose()
        << ")'" << std::endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(const EssentialMatrix& E, boost::optional<Matrix&> H =
      boost::none) const {
    return (Vector(1) << E.error(vA_, vB_, H));
  }

};

/**
 * Binary factor that optimizes for E and inverse depth: assumes measurement
 * in image 2 is perfect, and returns re-projection error in image 1
 */
class EssentialMatrixFactor2: public NoiseModelFactor2<EssentialMatrix,
    LieScalar> {

  Point3 dP1_; ///< 3D point corresponding to measurement in image 1
  Point2 p1_, p2_; ///< Measurements in image 1 and image 2
  Cal3_S2 K_; ///< Calibration

  typedef NoiseModelFactor2<EssentialMatrix, LieScalar> Base;
  typedef EssentialMatrixFactor2 This;

public:

  /// Constructor
  EssentialMatrixFactor2(Key key1, Key key2, const Point2& pA, const Point2& pB,
      const Cal3_S2& K, const SharedNoiseModel& model) :
      Base(model, key1, key2), p1_(pA), p2_(pB), K_(K) {
    Point2 xy = K_.calibrate(p1_);
    dP1_ = Point3(xy.x(), xy.y(), 1);
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast < gtsam::NonlinearFactor
        > (gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor2 with measurements\n  ("
        << p1_.vector().transpose() << ")' and (" << p2_.vector().transpose()
        << ")'" << std::endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(const EssentialMatrix& E, const LieScalar& d,
      boost::optional<Matrix&> DE = boost::none, boost::optional<Matrix&> Dd =
          boost::none) const {

    // We have point x,y in image 1
    // Given a depth Z, the corresponding 3D point P1 = Z*(x,y,1) = (x,y,1)/d
    // We then convert to first camera by 2P = 1R2�*(P1-1T2)
    // The homogeneous coordinates of can be written as
    //   2R1*(P1-1T2) ==  2R1*d*(P1-1T2) == 2R1*((x,y,1)-d*1T2)
    // Note that this is just a homography for d==0
    // The point d*P1 = (x,y,1) is computed in constructor as dP1_

    // Project to normalized image coordinates, then uncalibrate
    Point2 pi;
    if (!DE && !Dd) {

      Point3 _1T2 = E.direction().point3();
      Point3 d1T2 = d * _1T2;
      Point3 dP2 = E.rotation().unrotate(dP1_ - d1T2);
      Point2 pn = SimpleCamera::project_to_camera(dP2);
      pi = K_.uncalibrate(pn);

    } else {

      // Calculate derivatives. TODO if slow: optimize with Mathematica
      //     3*2        3*3       3*3        2*3      2*2
      Matrix D_1T2_dir, DdP2_rot, DP2_point, Dpn_dP2, Dpi_pn;

      Point3 _1T2 = E.direction().point3(D_1T2_dir);
      Point3 d1T2 = d * _1T2;
      Point3 dP2 = E.rotation().unrotate(dP1_ - d1T2, DdP2_rot, DP2_point);
      Point2 pn = SimpleCamera::project_to_camera(dP2, Dpn_dP2);
      pi = K_.uncalibrate(pn, boost::none, Dpi_pn);

      if (DE) {
        Matrix DdP2_E(3, 5);
        DdP2_E << DdP2_rot, -DP2_point * d * D_1T2_dir; // (3*3), (3*3) * (3*2)
        *DE = Dpi_pn * (Dpn_dP2 * DdP2_E); // (2*2) * (2*3) * (3*5)
      }

      if (Dd) // efficient backwards computation:
        //     (2*2)   * (2*3)    * (3*3)      * (3*1)
        *Dd = -(Dpi_pn * (Dpn_dP2 * (DP2_point * _1T2.vector())));

    }
    Point2 reprojectionError = pi - p2_;
    return reprojectionError.vector();
  }

};
// EssentialMatrixFactor2

}// gtsam

