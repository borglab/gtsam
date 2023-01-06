/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2014, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file EssentialMatrixFactor.h
 * @brief EssentialMatrixFactor class
 * @author Frank Dellaert
 * @author Ayush Baid
 * @author Akshay Krishnan
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>

namespace gtsam {

/**
 * Factor that evaluates epipolar error p'Ep for given essential matrix
 */
class EssentialMatrixFactor : public NoiseModelFactorN<EssentialMatrix> {
  Vector3 vA_, vB_;  ///< Homogeneous versions, in ideal coordinates

  typedef NoiseModelFactorN<EssentialMatrix> Base;
  typedef EssentialMatrixFactor This;

 public:
  using Base::evaluateError;
  /**
   *  Constructor
   *  @param key Essential Matrix variable key
   *  @param pA point in first camera, in calibrated coordinates
   *  @param pB point in second camera, in calibrated coordinates
   *  @param model noise model is about dot product in ideal, homogeneous
   * coordinates
   */
  EssentialMatrixFactor(Key key, const Point2& pA, const Point2& pB,
                        const SharedNoiseModel& model)
      : Base(model, key) {
    vA_ = EssentialMatrix::Homogeneous(pA);
    vB_ = EssentialMatrix::Homogeneous(pB);
  }

  /**
   *  Constructor
   *  @param key Essential Matrix variable key
   *  @param pA point in first camera, in pixel coordinates
   *  @param pB point in second camera, in pixel coordinates
   *  @param model noise model is about dot product in ideal, homogeneous
   * coordinates
   *  @param K calibration object, will be used only in constructor
   */
  template <class CALIBRATION>
  EssentialMatrixFactor(Key key, const Point2& pA, const Point2& pB,
                        const SharedNoiseModel& model,
                        boost::shared_ptr<CALIBRATION> K)
      : Base(model, key) {
    assert(K);
    vA_ = EssentialMatrix::Homogeneous(K->calibrate(pA));
    vB_ = EssentialMatrix::Homogeneous(K->calibrate(pB));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor with measurements\n  ("
              << vA_.transpose() << ")' and (" << vB_.transpose() << ")'"
              << std::endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(
      const EssentialMatrix& E, OptionalMatrixType H) const override {
    Vector error(1);
    error << E.error(vA_, vB_, H);
    return error;
  }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Binary factor that optimizes for E and inverse depth d: assumes measurement
 * in image 2 is perfect, and returns re-projection error in image 1
 */
class EssentialMatrixFactor2
    : public NoiseModelFactorN<EssentialMatrix, double> {
  Point3 dP1_;  ///< 3D point corresponding to measurement in image 1
  Point2 pn_;   ///< Measurement in image 2, in ideal coordinates
  double f_;    ///< approximate conversion factor for error scaling

  typedef NoiseModelFactorN<EssentialMatrix, double> Base;
  typedef EssentialMatrixFactor2 This;

 public:
  using Base::evaluateError;
  /**
   *  Constructor
   *  @param key1 Essential Matrix variable key
   *  @param key2 Inverse depth variable key
   *  @param pA point in first camera, in calibrated coordinates
   *  @param pB point in second camera, in calibrated coordinates
   *  @param model noise model should be in pixels, as well
   */
  EssentialMatrixFactor2(Key key1, Key key2, const Point2& pA, const Point2& pB,
                         const SharedNoiseModel& model)
      : Base(model, key1, key2),
        dP1_(EssentialMatrix::Homogeneous(pA)),
        pn_(pB) {
    f_ = 1.0;
  }

  /**
   *  Constructor
   *  @param key1 Essential Matrix variable key
   *  @param key2 Inverse depth variable key
   *  @param pA point in first camera, in pixel coordinates
   *  @param pB point in second camera, in pixel coordinates
   *  @param K calibration object, will be used only in constructor
   *  @param model noise model should be in pixels, as well
   */
  template <class CALIBRATION>
  EssentialMatrixFactor2(Key key1, Key key2, const Point2& pA, const Point2& pB,
                         const SharedNoiseModel& model,
                         boost::shared_ptr<CALIBRATION> K)
      : Base(model, key1, key2),
        dP1_(EssentialMatrix::Homogeneous(K->calibrate(pA))),
        pn_(K->calibrate(pB)) {
    f_ = 0.5 * (K->fx() + K->fy());
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor2 with measurements\n  ("
              << dP1_.transpose() << ")' and (" << pn_.transpose() << ")'"
              << std::endl;
  }

  /*
   * Vector of errors returns 2D vector
   * @param E essential matrix
   * @param d inverse depth d
   */
  Vector evaluateError(
      const EssentialMatrix& E, const double& d,
      OptionalMatrixType DE, OptionalMatrixType Dd) const override {
    // We have point x,y in image 1
    // Given a depth Z, the corresponding 3D point P1 = Z*(x,y,1) = (x,y,1)/d
    // We then convert to second camera by P2 = 1R2'*(P1-1T2)
    // The homogeneous coordinates of can be written as
    //   2R1*(P1-1T2) ==  2R1*d*(P1-1T2) == 2R1*((x,y,1)-d*1T2)
    // where we multiplied with d which yields equivalent homogeneous
    // coordinates. Note that this is just the homography 2R1 for d==0 The point
    // d*P1 = (x,y,1) is computed in constructor as dP1_

    // Project to normalized image coordinates, then uncalibrate
    Point2 pn(0, 0);
    if (!DE && !Dd) {
      Point3 _1T2 = E.direction().point3();
      Point3 d1T2 = d * _1T2;
      Point3 dP2 = E.rotation().unrotate(dP1_ - d1T2);  // 2R1*((x,y,1)-d*1T2)
      pn = PinholeBase::Project(dP2);

    } else {
      // Calculate derivatives. TODO if slow: optimize with Mathematica
      //     3*2        3*3       3*3
      Matrix D_1T2_dir, DdP2_rot, DP2_point;

      Point3 _1T2 = E.direction().point3(D_1T2_dir);
      Point3 d1T2 = d * _1T2;
      Point3 dP2 = E.rotation().unrotate(dP1_ - d1T2, DdP2_rot, DP2_point);

      Matrix23 Dpn_dP2;
      pn = PinholeBase::Project(dP2, Dpn_dP2);

      if (DE) {
        Matrix DdP2_E(3, 5);
        DdP2_E << DdP2_rot, -DP2_point * d * D_1T2_dir;  // (3*3), (3*3) * (3*2)
        *DE = f_ * Dpn_dP2 * DdP2_E;                     // (2*3) * (3*5)
      }

      if (Dd)  // efficient backwards computation:
        //      (2*3)    * (3*3)      * (3*1)
        *Dd = -f_ * (Dpn_dP2 * (DP2_point * _1T2));
    }
    Point2 reprojectionError = pn - pn_;
    return f_ * reprojectionError;
  }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// EssentialMatrixFactor2

/**
 * Binary factor that optimizes for E and inverse depth d: assumes measurement
 * in image 2 is perfect, and returns re-projection error in image 1
 * This version takes an extrinsic rotation to allow for omni-directional rigs
 */
class EssentialMatrixFactor3 : public EssentialMatrixFactor2 {
  typedef EssentialMatrixFactor2 Base;
  typedef EssentialMatrixFactor3 This;

  Rot3 cRb_;  ///< Rotation from body to camera frame

 public:
  using Base::evaluateError;
  /**
   *  Constructor
   *  @param key1 Essential Matrix variable key
   *  @param key2 Inverse depth variable key
   *  @param pA point in first camera, in calibrated coordinates
   *  @param pB point in second camera, in calibrated coordinates
   *  @param bRc extra rotation between "body" and "camera" frame
   *  @param model noise model should be in calibrated coordinates, as well
   */
  EssentialMatrixFactor3(Key key1, Key key2, const Point2& pA, const Point2& pB,
                         const Rot3& cRb, const SharedNoiseModel& model)
      : EssentialMatrixFactor2(key1, key2, pA, pB, model), cRb_(cRb) {}

  /**
   *  Constructor
   *  @param key1 Essential Matrix variable key
   *  @param key2 Inverse depth variable key
   *  @param pA point in first camera, in pixel coordinates
   *  @param pB point in second camera, in pixel coordinates
   *  @param K calibration object, will be used only in constructor
   *  @param model noise model should be in pixels, as well
   */
  template <class CALIBRATION>
  EssentialMatrixFactor3(Key key1, Key key2, const Point2& pA, const Point2& pB,
                         const Rot3& cRb, const SharedNoiseModel& model,
                         boost::shared_ptr<CALIBRATION> K)
      : EssentialMatrixFactor2(key1, key2, pA, pB, model, K), cRb_(cRb) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor3 with rotation " << cRb_ << std::endl;
  }

  /*
   * Vector of errors returns 2D vector
   * @param E essential matrix
   * @param d inverse depth d
   */
  Vector evaluateError(
      const EssentialMatrix& E, const double& d,
      OptionalMatrixType DE, OptionalMatrixType Dd) const override {
    if (!DE) {
      // Convert E from body to camera frame
      EssentialMatrix cameraE = cRb_ * E;
      // Evaluate error
      return Base::evaluateError(cameraE, d, OptionalNone, Dd);
    } else {
      // Version with derivatives
      Matrix D_e_cameraE, D_cameraE_E;  // 2*5, 5*5
      EssentialMatrix cameraE = E.rotate(cRb_, D_cameraE_E);
      Vector e = Base::evaluateError(cameraE, d, D_e_cameraE, Dd);
      *DE = D_e_cameraE * D_cameraE_E;  // (2*5) * (5*5)
      return e;
    }
  }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// EssentialMatrixFactor3

/**
 * Binary factor that optimizes for E and calibration K using the algebraic
 * epipolar error (K^-1 pA)'E (K^-1 pB). The calibration is shared between two
 * images.
 *
 * Note: As correspondences between 2d coordinates can only recover 7 DoF,
 * this factor should always be used with a prior factor on calibration.
 * Even with a prior, we can only optimize 2 DoF in the calibration. So the
 * prior should have a noise model with very low sigma in the remaining
 * dimensions. This has been tested to work on Cal3_S2. With Cal3Bundler, it
 * works only with a strong prior (low sigma noisemodel) on all degrees of
 * freedom.
 */
template <class CALIBRATION>
class EssentialMatrixFactor4
    : public NoiseModelFactorN<EssentialMatrix, CALIBRATION> {
 private:
  Point2 pA_, pB_;  ///< points in pixel coordinates

  typedef NoiseModelFactorN<EssentialMatrix, CALIBRATION> Base;
  typedef EssentialMatrixFactor4 This;

  static constexpr int DimK = FixedDimension<CALIBRATION>::value;
  typedef Eigen::Matrix<double, 2, DimK> JacobianCalibration;

 public:
  using Base::evaluateError;
  /**
   *  Constructor
   *  @param keyE Essential Matrix (from camera B to A) variable key
   *  @param keyK Calibration variable key (common for both cameras)
   *  @param pA point in first camera, in pixel coordinates
   *  @param pB point in second camera, in pixel coordinates
   *  @param model noise model is about dot product in ideal, homogeneous
   * coordinates
   */
  EssentialMatrixFactor4(Key keyE, Key keyK, const Point2& pA, const Point2& pB,
                         const SharedNoiseModel& model)
      : Base(model, keyE, keyK), pA_(pA), pB_(pB) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor4 with measurements\n  ("
              << pA_.transpose() << ")' and (" << pB_.transpose() << ")'"
              << std::endl;
  }

  /**
   * @brief Calculate the algebraic epipolar error pA' (K^-1)' E K pB.
   *
   * @param E essential matrix for key keyE
   * @param K calibration (common for both images) for key keyK
   * @param H1 optional jacobian of error w.r.t E
   * @param H2 optional jacobian of error w.r.t K
   * @return * Vector 1D vector of algebraic error
   */
  Vector evaluateError(
      const EssentialMatrix& E, const CALIBRATION& K,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {
    // converting from pixel coordinates to normalized coordinates cA and cB
    JacobianCalibration cA_H_K;  // dcA/dK
    JacobianCalibration cB_H_K;  // dcB/dK
    Point2 cA = K.calibrate(pA_, H2 ? &cA_H_K : 0, OptionalNone);
    Point2 cB = K.calibrate(pB_, H2 ? &cB_H_K : 0, OptionalNone);

    // convert to homogeneous coordinates
    Vector3 vA = EssentialMatrix::Homogeneous(cA);
    Vector3 vB = EssentialMatrix::Homogeneous(cB);

    if (H2) {
      // compute the jacobian of error w.r.t K

      // error function f = vA.T * E * vB
      // H2 = df/dK = vB.T * E.T * dvA/dK + vA.T * E * dvB/dK
      // where dvA/dK = dvA/dcA * dcA/dK, dVB/dK = dvB/dcB * dcB/dK
      // and dvA/dcA = dvB/dcB = [[1, 0], [0, 1], [0, 0]]
      *H2 = vB.transpose() * E.matrix().transpose().leftCols<2>() * cA_H_K +
            vA.transpose() * E.matrix().leftCols<2>() * cB_H_K;
    }

    Vector error(1);
    error << E.error(vA, vB, H1);

    return error;
  }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// EssentialMatrixFactor4

}  // namespace gtsam
