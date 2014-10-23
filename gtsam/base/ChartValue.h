/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ChartValue.h
 * @brief A template class wrapper for values to be constructed from any type T that has a DefalutChart<T>.
 * @author Michael Bosse and Paul Furgale
 * @date Oct 22, 2014
 */

#pragma once


#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

template<typename T>
class ChartValue : public DerivedValue< ChartValue<T> > {
  DefaultChart<T> chart_;
 public:
  enum { dimension = traits::dimension<T>::value };

  typedef DefaultChart<T> Chart;

  ChartValue() : chart_(traits::zero<T>::value()) {}
  ChartValue( const T& value) : chart_(value) {}
  ChartValue( const DefaultChart<T>& chart) : chart_(chart) {}
  virtual ~ChartValue() {}

  // print with optional string (virtual, implements Value::print()
  virtual void print(const std::string& s = "") const {
    chart_.print(s);
  }

  bool equals(const ChartValue<T>& other, double tol = 1e-9) const {
    typename Chart::vector delta = chart_.apply(other.chart_.t_);
    return delta.cwiseAbs().maxCoeff() < tol;
  }

  // Tangent space dimensionality (virtual, implements Value::dim())
  virtual size_t dim() const {
    return dimension;
    //todo: how should this work if the dimension is Dynamic?
  }

  // retract working directly with Rot3 objects (non-virtual, non-overriding!)
  T retract(const Vector& delta) const {
    return chart_.retract(delta);
  }

  // localCoordinates working directly with T objects (non-virtual, non-overriding!)
  Vector localCoordinates(const ChartValue<T>& other) const {
    return chart_.apply(other.chart_.origin());
  }

  const T& origin(boost::optional<Eigen::Matrix<double,dimension,dimension>&> H=boost::none ) const {
    if (H) {
      H->setIdentity();
    }
    return chart_.origin();
  }
};
// This is used for an expression to covert the chartvalue to the origin
template<typename T>
const T& chart_origin(const ChartValue<T>& value,boost::optional<Eigen::Matrix<double,ChartValue<T>::dimension,ChartValue<T>::dimension>&> H=boost::none) {
  return value.origin(H);
}

// setup default traits specializations for a Chart Value
namespace traits {
template<typename T>
struct is_group<ChartValue<T> > : public std::true_type {
};
template<typename T>
struct is_manifold<ChartValue<T> > : public std::true_type {
};
template<typename T>
struct dimension<ChartValue<T> > : public std::integral_constant<int, ChartValue<T>::dimension> {
};
}
}
