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

/// Additive Group
template<typename Derived>
class AdditiveGroup {

};

/// Cyclic group of order N
template<size_t N>
class Cyclic : AdditiveGroup<Cyclic<N> > {
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
  /// print with optional string
  void print(const std::string& s = "") const {
    std::cout << s << i_ << std::endl;
  }

  /// equals with an tolerance, prints out message if unequal
  bool equals(const Cyclic& other, double tol = 1e-9) const {
    return other.i_ == i_;
  }

};

namespace traits {
/// Define Cyclic to be a model of the Group concept
template<size_t N> struct structure_category<Cyclic<N> > {
  typedef group_tag type;
};
} // \namespace gtsam::traits

namespace group {

template<typename G>
AdditiveGroup<G> compose(const AdditiveGroup<G>&g, const AdditiveGroup<G>& h) {
  return g + h;
}

template<typename G>
AdditiveGroup<G> between(const AdditiveGroup<G>&g, const AdditiveGroup<G>& h) {
  return h - g;
}

template<typename G>
AdditiveGroup<G> inverse(const AdditiveGroup<G>&g) {
  return -g;
}

namespace traits {

/// Define the trait that specifies Cyclic's identity element
template<size_t N> struct identity<Cyclic<N> > {
  static const Cyclic<N> value;
  typedef Cyclic<N> value_type;
};

template<size_t N>
const Cyclic<N> identity<Cyclic<N> >::value = Cyclic<N>(0);

/// Define the trait that asserts AdditiveGroup is an additive group
template<typename G> struct flavor<AdditiveGroup<G> > {
  typedef additive_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

