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

#include <boost/random/mersenne_twister.hpp>

#include <iosfwd>
#include <string>

namespace gtsam {

using SO4 = SO<4>;

// /// Random SO(4) element (no big claims about uniformity)
// static SO4 Random(boost::mt19937 &rng);

// Below are all declarations of SO<4> specializations.
// They are *defined* in SO4.cpp.

template <>
Matrix4 SO4::Hat(const TangentVector& xi);

template <>
Vector6 SO4::Vee(const Matrix4& X);

template <>
SO4 SO4::Expmap(const Vector6& xi, ChartJacobian H);

template <>
Matrix6 SO4::AdjointMap() const;

template <>
SO4::VectorN2 SO4::vec(OptionalJacobian<16, 6> H) const;

template <>
SO4 SO4::ChartAtOrigin::Retract(const Vector6& omega, ChartJacobian H);

template <>
Vector6 SO4::ChartAtOrigin::Local(const SO4& R, ChartJacobian H);

/**
 * Project to top-left 3*3 matrix. Note this is *not* in general \in SO(3).
 */
Matrix3 topLeft(const SO4& Q, OptionalJacobian<9, 6> H = boost::none);

/**
 * Project to Stiefel manifold of 4*3 orthonormal 3-frames in R^4, i.e., pi(Q)
 * -> S \in St(3,4).
 */
Matrix43 stiefel(const SO4& Q, OptionalJacobian<12, 6> H = boost::none);

//  private:
//   /** Serialization function */
//   friend class boost::serialization::access;
//   template <class ARCHIVE>
//   void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
//     ar &boost::serialization::make_nvp("Q11", (*this)(0, 0));
//     ar &boost::serialization::make_nvp("Q12", (*this)(0, 1));
//     ar &boost::serialization::make_nvp("Q13", (*this)(0, 2));
//     ar &boost::serialization::make_nvp("Q14", (*this)(0, 3));

//     ar &boost::serialization::make_nvp("Q21", (*this)(1, 0));
//     ar &boost::serialization::make_nvp("Q22", (*this)(1, 1));
//     ar &boost::serialization::make_nvp("Q23", (*this)(1, 2));
//     ar &boost::serialization::make_nvp("Q24", (*this)(1, 3));

//     ar &boost::serialization::make_nvp("Q31", (*this)(2, 0));
//     ar &boost::serialization::make_nvp("Q32", (*this)(2, 1));
//     ar &boost::serialization::make_nvp("Q33", (*this)(2, 2));
//     ar &boost::serialization::make_nvp("Q34", (*this)(2, 3));

//     ar &boost::serialization::make_nvp("Q41", (*this)(3, 0));
//     ar &boost::serialization::make_nvp("Q42", (*this)(3, 1));
//     ar &boost::serialization::make_nvp("Q43", (*this)(3, 2));
//     ar &boost::serialization::make_nvp("Q44", (*this)(3, 3));
//   }

/*
 * Define the traits. internal::LieGroup provides both Lie group and Testable
 */

template <>
struct traits<SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

template <>
struct traits<const SO4> : Testable<SO4>, internal::LieGroupTraits<SO4> {};

}  // end namespace gtsam
