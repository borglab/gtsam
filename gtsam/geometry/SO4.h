/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO4.h
 * @brief   4*4 matrix representation of SO(4)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    March 2019
 */

#pragma once

#include <gtsam/geometry/SOn.h>

#include <gtsam/base/Group.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <string>

namespace gtsam {

using SO4 = SO<4>;

// /// Random SO(4) element (no big claims about uniformity)
// static SO4 Random(std::mt19937 &rng);

// Below are all declarations of SO<4> specializations.
// They are *defined* in SO4.cpp.

template <>
GTSAM_EXPORT
Matrix4 SO4::Hat(const TangentVector &xi);

template <>
GTSAM_EXPORT
Vector6 SO4::Vee(const Matrix4 &X);

template <>
GTSAM_EXPORT
SO4 SO4::Expmap(const Vector6 &xi, ChartJacobian H);

template <>
GTSAM_EXPORT
Matrix6 SO4::AdjointMap() const;

template <>
GTSAM_EXPORT
SO4::VectorN2 SO4::vec(OptionalJacobian<16, 6> H) const;

template <>
GTSAM_EXPORT
SO4 SO4::ChartAtOrigin::Retract(const Vector6 &omega, ChartJacobian H);

template <>
GTSAM_EXPORT
Vector6 SO4::ChartAtOrigin::Local(const SO4 &Q, ChartJacobian H);

/**
 * Project to top-left 3*3 matrix. Note this is *not* in general \in SO(3).
 */
GTSAM_EXPORT Matrix3 topLeft(const SO4 &Q, OptionalJacobian<9, 6> H = {});

/**
 * Project to Stiefel manifold of 4*3 orthonormal 3-frames in R^4, i.e., pi(Q)
 * -> \f$ S \in St(3,4) \f$.
 */
GTSAM_EXPORT Matrix43 stiefel(const SO4 &Q, OptionalJacobian<12, 6> H = {});

template <class Archive>
/** Serialization function */
void serialize(Archive &ar, SO4 &Q, const unsigned int /*version*/) {
  Matrix4 &M = Q.matrix_;
  ar &boost::serialization::make_nvp("Q11", M(0, 0));
  ar &boost::serialization::make_nvp("Q12", M(0, 1));
  ar &boost::serialization::make_nvp("Q13", M(0, 2));
  ar &boost::serialization::make_nvp("Q14", M(0, 3));

  ar &boost::serialization::make_nvp("Q21", M(1, 0));
  ar &boost::serialization::make_nvp("Q22", M(1, 1));
  ar &boost::serialization::make_nvp("Q23", M(1, 2));
  ar &boost::serialization::make_nvp("Q24", M(1, 3));

  ar &boost::serialization::make_nvp("Q31", M(2, 0));
  ar &boost::serialization::make_nvp("Q32", M(2, 1));
  ar &boost::serialization::make_nvp("Q33", M(2, 2));
  ar &boost::serialization::make_nvp("Q34", M(2, 3));

  ar &boost::serialization::make_nvp("Q41", M(3, 0));
  ar &boost::serialization::make_nvp("Q42", M(3, 1));
  ar &boost::serialization::make_nvp("Q43", M(3, 2));
  ar &boost::serialization::make_nvp("Q44", M(3, 3));
}

/*
 * Define the traits. internal::LieGroup provides both Lie group and Testable
 */

template <>
struct traits<SO4> : public internal::LieGroup<SO4> {};

template <>
struct traits<const SO4> : public internal::LieGroup<SO4> {};

}  // end namespace gtsam
