/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Point1.h
 * @brief   1D Point
 * @author  Sven Lilge
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/std_optional_serialization.h>
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace gtsam {

typedef Vector1 Point1;

/// Convenience typedefs
using Point1Pair = std::pair<Point1, Point1>;
using Point1Pairs = std::vector<Point1Pair>;

/// Stream insertion operator for Point1Pair.
GTSAM_EXPORT std::ostream& operator<<(std::ostream& os,
                                      const gtsam::Point1Pair& p);

/// L1 norm (absolute value) of a Point1, with optional Jacobian.
GTSAM_EXPORT double norm1(const Point1& p, OptionalJacobian<1, 1> H = {});

/// Distance between two 1D points, with optional Jacobians.
GTSAM_EXPORT double distance1(const Point1& p1, const Point1& q,
                              OptionalJacobian<1, 1> H1 = {},
                              OptionalJacobian<1, 1> H2 = {});

template <typename A1, typename A2>
struct Range;

/// Range functor between two Point1 values, using L1 distance.
template <>
struct Range<Point1, Point1> {
  typedef double result_type;
  double operator()(const Point1& p, const Point1& q,
                    OptionalJacobian<1, 1> H1 = {},
                    OptionalJacobian<1, 1> H2 = {}) {
    return distance1(p, q, H1, H2);
  }
};

}  // namespace gtsam
