/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EssentialMatrixWithCalibrationFactor.h
 *
 * @brief A factor evaluating algebraic epipolar error with essential matrix and calibration as variables.
 *
 * @author Ayush Baid
 * @author Akshay Krishnan
 * @date April 23, 2021
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <iostream>

namespace gtsam {

/**
 * Factor that evaluates algebraic epipolar error (K^-1 p)'E (K^-1 p) for given essential matrix and calibration shared
 * between two images.
 */
template<class CALIBRATION>
class EssentialMatrixWithCalibrationFactor: public NoiseModelFactor2<EssentialMatrix, CALIBRATION > {

  Point2 pA_, pB_; ///< points in pixel coordinates

  typedef NoiseModelFactor2<EssentialMatrix, CALIBRATION> Base;
  typedef EssentialMatrixWithCalibrationFactor This;

public:

  /**
   *  Constructor
   *  @param essentialMatrixKey Essential Matrix variable key
   *  @param calibrationKey Calibration variable key
   *  @param pA point in first camera, in pixel coordinates
   *  @param pB point in second camera, in pixel coordinates
   *  @param model noise model is about dot product in ideal, homogeneous coordinates
   */
  EssentialMatrixWithCalibrationFactor(Key essentialMatrixKey, Key calibrationKey, const Point2& pA, const Point2& pB,
      const SharedNoiseModel& model) :
      Base(model, essentialMatrixKey, calibrationKey), pA_(pA), pB_(pB) {}


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s);
    std::cout << "  EssentialMatrixWithCalibrationFactor with measurements\n  ("
        << pA_.transpose() << ")' and (" << pB_.transpose() << ")'"
        << std::endl;
  }

  /// vector of errors returns 1D vector
  /**
   * @brief Calculate the algebraic epipolar error p' (K^-1)' E K p.
   *
   * @param E essential matrix for key essentialMatrixKey
   * @param K calibration (common for both images) for key calibrationKey
   * @param H1 optional jacobian in E
   * @param H2 optional jacobian in K
   * @return * Vector
   */
  Vector evaluateError(const EssentialMatrix& E, const CALIBRATION& K,
    boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const override {
    Vector error(1);
    // converting from pixel coordinates to normalized coordinates cA and cB
    Matrix cA_H_K; // dcA/dK
    Matrix cB_H_K; // dcB/dK
    Point2 cA = K.calibrate(pA_, cA_H_K);
    Point2 cB = K.calibrate(pB_, cB_H_K);

    // Homogeneous the coordinates
    Vector3 vA = EssentialMatrix::Homogeneous(cA);
    Vector3 vB = EssentialMatrix::Homogeneous(cB);

    if (H2){
      // compute the jacobian of error w.r.t K

      // dvX / dcX [3x2] = [1, 0], [0, 1], [0, 0]
      Matrix v_H_c = (Matrix(3, 2) << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(); // [3x2]

      // computing dvA/dK = dvA/dcA * dcA/dK and dVB/dK = dvB/dcB * dcB/dK
      Matrix vA_H_K = v_H_c * cA_H_K;
      Matrix vB_H_K = v_H_c * cB_H_K;

      // error function f = vB.T * E * vA
      // H2 = df/dK = vB.T * E.T * dvA/dK + vA.T * E * dvB/dK
      *H2 = vB.transpose() * E.matrix().transpose() * vA_H_K + vA.transpose() * E.matrix() * vB_H_K;
    }

    error << E.error(vA, vB, H1);

    return error;
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

}// gtsam
