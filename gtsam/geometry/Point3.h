/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Point3.h
 * @brief  3D Point
 * @author Alireza Fathi
 * @author Christian Potthast
 * @author Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/config.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <boost/serialization/nvp.hpp>
#include <numeric>

namespace gtsam {

/// As of GTSAM 4, in order to make GTSAM more lean,
/// it is now possible to just typedef Point3 to Vector3
typedef Vector3 Point3;

// Convenience typedef
using Point3Pair = std::pair<Point3, Point3>;
GTSAM_EXPORT std::ostream &operator<<(std::ostream &os, const gtsam::Point3Pair &p);

using Point3Pairs = std::vector<Point3Pair>;

/// distance between two points
GTSAM_EXPORT double distance3(const Point3& p1, const Point3& q,
	                          OptionalJacobian<1, 3> H1 = boost::none,
                              OptionalJacobian<1, 3> H2 = boost::none);

/// Distance of the point from the origin, with Jacobian
GTSAM_EXPORT double norm3(const Point3& p, OptionalJacobian<1, 3> H = boost::none);

/// normalize, with optional Jacobian
GTSAM_EXPORT Point3 normalize(const Point3& p, OptionalJacobian<3, 3> H = boost::none);

/// cross product @return this x q
GTSAM_EXPORT Point3 cross(const Point3& p, const Point3& q,
                          OptionalJacobian<3, 3> H_p = boost::none,
                          OptionalJacobian<3, 3> H_q = boost::none);

/// dot product
GTSAM_EXPORT double dot(const Point3& p, const Point3& q,
                        OptionalJacobian<1, 3> H_p = boost::none,
                        OptionalJacobian<1, 3> H_q = boost::none);

/// mean
template <class CONTAINER>
Point3 mean(const CONTAINER& points) {
  if (points.size() == 0) throw std::invalid_argument("Point3::mean input container is empty");
  Point3 sum(0, 0, 0);
  sum = std::accumulate(points.begin(), points.end(), sum);
  return sum / points.size();
}

/// Calculate the two means of a set of Point3 pairs
GTSAM_EXPORT Point3Pair means(const std::vector<Point3Pair> &abPointPairs);

template <typename A1, typename A2>
struct Range;

template <>
struct Range<Point3, Point3> {
  typedef double result_type;
  double operator()(const Point3& p, const Point3& q,
                    OptionalJacobian<1, 3> H1 = boost::none,
                    OptionalJacobian<1, 3> H2 = boost::none) {
    return distance3(p, q, H1, H2);
  }
};

}  // namespace gtsam

