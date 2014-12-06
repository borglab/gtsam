/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cyclic.h
 * @brief  Cyclic group, i.e., the integers modulo N
 * @author Frank Dellaert
 **/

#include <gtsam/base/concepts.h>
#include <cstddef>

namespace gtsam {

template<size_t N>
class Cyclic {
  size_t i_; ///< we just use an unsigned int
public:
  /// Constructor
  Cyclic(size_t i) :
      i_(i) {
  }
  /// Cast to size_t
  operator size_t() const {
    return i_;
  }
  /// Addition modulo N
  Cyclic operator+(const Cyclic& h) const {
    return (i_ + h.i_) % N;
  }
  /// Subtraction modulo N
  Cyclic operator-(const Cyclic& h) const {
    return (N + i_ - h.i_) % N;
  }
  /// Inverse
  Cyclic operator-() const {
    return (N - i_) % N;
  }
};

namespace traits {
/// Define Cyclic to be a model of the Group concept
template<size_t N> struct structure_category<Cyclic<N> > {
  typedef group_tag type;
};
} // \namespace gtsam::traits

namespace group {

template<size_t N>
Cyclic<N> compose(const Cyclic<N>&g, const Cyclic<N>& h) {
  return g + h;
}

template<size_t N>
Cyclic<N> between(const Cyclic<N>&g, const Cyclic<N>& h) {
  return h - g;
}

template<size_t N>
Cyclic<N> inverse(const Cyclic<N>&g) {
  return -g;
}

namespace traits {

/// Define the trait that specifies Cyclic's identity element
template<size_t N> struct identity<Cyclic<N> > {
  static const Cyclic<N> value = Cyclic<N>(0);
  typedef Cyclic<N> value_type;
};

/// Define the trait that asserts Cyclic is an additive group
template<size_t N> struct flavor<Cyclic<N> > {
  typedef additive_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

