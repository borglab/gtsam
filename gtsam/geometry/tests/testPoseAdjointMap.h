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

  // Create a covariance matrix for type T. Use sigma_values^2 on the diagonal
  // and fill in non-diagonal entries with correlation coefficient of 1. Note:
  // a covariance matrix for T has the same dimensions as a Jacobian for T.
  template<class T>
  typename T::Jacobian GenerateFullCovariance(
    std::array<double, T::dimension> sigma_values)
  {
    typename T::TangentVector sigmas(&sigma_values.front());
    return typename T::Jacobian{sigmas * sigmas.transpose()};
  }

  // Create a covariance matrix with one non-zero element on the diagonal.
  template<class T>
  typename T::Jacobian GenerateOneVariableCovariance(int idx, double sigma)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  // Create a covariance matrix with two non-zero elements on the diagonal with
  // a correlation of 1.0
  template<class T>
  typename T::Jacobian GenerateTwoVariableCovariance(int idx0, int idx1, double sigma0, double sigma1)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }

  // Overloaded function to create a Rot2 from one angle.
  Rot2 RotFromArray(const std::array<double, Rot2::dimension> &r)
  {
    return Rot2{r[0] * degree};
  }

  // Overloaded function to create a Rot3 from three angles.
  Rot3 RotFromArray(const std::array<double, Rot3::dimension> &r)
  {
    return Rot3::RzRyRx(r[0] * degree, r[1] * degree, r[2] * degree);
  }

  // Transform a covariance matrix with a rotation and a translation
  template<class Pose>
  typename Pose::Jacobian RotateTranslate(
    std::array<double, Pose::Rotation::dimension> r,
    std::array<double, Pose::Translation::dimension> t,
    const typename Pose::Jacobian &cov)
  {
    // Construct a pose object
    typename Pose::Rotation rot{RotFromArray(r)};
    Pose wTb{rot, typename Pose::Translation{&t.front()}};

    // transform the covariance with the AdjointMap
    auto adjointMap = wTb.AdjointMap();
    return adjointMap * cov * adjointMap.transpose();
  }
}