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

/// Additive Group, using CRTP
template<typename Derived>
struct AdditiveGroup {
  static Derived Identity() {
    return Derived::Identity();
  }
  Derived const * derived() const {
    return static_cast<Derived const*>(this);
  }
  Derived operator+(const AdditiveGroup& h) const {
    return derived()->operator+(*h.derived());
  }
  Derived operator-(const AdditiveGroup& h) const {
    return derived()->operator-(*h.derived());
  }
  Derived operator-() const {
    return derived()->operator-();
  }
};

/// Cyclic group of order N
template<size_t N>
class Cyclic : public AdditiveGroup<Cyclic<N> > {
  size_t i_; ///< we just use an unsigned int
public:
  /// Constructor
  Cyclic(size_t i) :
      i_(i) {
  }
  // Idenity element
  static Cyclic Identity() {
    return Cyclic(0);
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
/// Define any additive group to be at least a model of the Group concept
template<typename G>
struct structure_category<AdditiveGroup<G> > {
  typedef group_tag type;
};
} // \namespace gtsam::traits

namespace group {

template<typename G>
G compose(const AdditiveGroup<G>&g, const AdditiveGroup<G>& h) {
  return g + h;
}

template<typename G>
G between(const AdditiveGroup<G>&g, const AdditiveGroup<G>& h) {
  return h - g;
}

template<typename G>
G inverse(const AdditiveGroup<G>&g) {
  return -g;
}

namespace traits {

/// Define the trait that specifies Cyclic's identity element
template<typename G> struct identity<AdditiveGroup<G> > {
  static const G value;
  typedef G value_type;
};

template<typename G>
const G identity<AdditiveGroup<G> >::value = AdditiveGroup<G>::Identity();

/// Define the trait that asserts AdditiveGroup is an additive group
template<typename G> struct flavor<AdditiveGroup<G> > {
  typedef additive_tag type;
};

} // \namespace gtsam::group::traits
} // \namespace gtsam::group
} // \namespace gtsam

