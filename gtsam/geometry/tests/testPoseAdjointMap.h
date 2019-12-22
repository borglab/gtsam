/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPoseAdjointMap.h
 * @brief   Support utilities for using AdjointMap for transforming Pose2 and Pose3 covariance matrices
 * @author  Peter Mulllen
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace test_pose_adjoint_map
{
  const double degree = M_PI / 180;

  // Return a covariance matrix for type T with non-zero values for every element.
  // Use sigma_values^2 on the diagonal and fill in non-diagonal entries with
  // correlation coefficient of 1. Note: a covariance matrix for T has the same
  // dimensions as a Jacobian for T, the returned matrix is not a Jacobian.
  template<class T>
  typename T::Jacobian FullCovarianceFromSigmas(
    const typename T::TangentVector &sigmas)
  {
    return sigmas * sigmas.transpose();
  }

  // Return a covariance matrix with one non-zero element on the diagonal.
  template<class T>
  typename T::Jacobian SingleVariableCovarianceFromSigma(int idx, double sigma)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  // Return a covariance matrix with two non-zero elements on the diagonal and
  // a correlation of 1.0 between the two variables.
  template<class T>
  typename T::Jacobian TwoVariableCovarianceFromSigmas(int idx0, int idx1, double sigma0, double sigma1)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }
}