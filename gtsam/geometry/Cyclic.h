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

#include <gtsam/base/Group.h>
#include <gtsam/base/Testable.h>

#include <cassert>
#include <iostream>  // for cout :-(

namespace gtsam {

/// Cyclic group of order N
template<size_t N>
class Cyclic {
  size_t i_; ///< we just use an unsigned int
public:
  /// Constructor
  Cyclic(size_t i) :
      i_(i) {
    assert(i < N);
  }
  /// Default constructor yields identity
  Cyclic():i_(0) {
  }
  static Cyclic identity() { return Cyclic();}

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

/// Define cyclic group to be a model of the Additive Group concept
template<size_t N>
struct traits<Cyclic<N> > : internal::AdditiveGroupTraits<Cyclic<N> >, //
    Testable<Cyclic<N> > {
};

} // \namespace gtsam

