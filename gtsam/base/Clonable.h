/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Clonable.h
 * @brief   Pure virtual class and CRTP implementation of clone()
 * @author  Jose Luis Blanco Claraco
 * @date    Sep 23, 2020
 */

#pragma once

#include <boost/make_shared.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/src/Core/util/Memory.h> // aligned_allocator

namespace gtsam {

/**
 * Virtual class imposing a uniform clone() as a base class.
 * 
 * Normal use is: use Clonable<T> as *virtual* base class of the polymorphic
 * base class, then ClonableImpl<U,T> as public base of each derived class.
 * @addtogroup base
 */
template<typename Base>
class Clonable {
public:
  /** shared pointer type to Base */
  using shared_ptr = boost::shared_ptr<Base>;

  /// @return a deep copy of this object
  virtual shared_ptr clone() const = 0;
};

/**
 * CRTP base class to implement Clonable<T> in each class U derived from T.
 * 
 * @addtogroup base
 */
template<typename Derived, typename Base=Derived>
class ClonableImpl: virtual Clonable<Base> {
public:
  /// @return a deep copy of this object
  boost::shared_ptr<Base> clone() const override {
    return boost::shared_ptr<Base>(
          new Derived(static_cast<const Derived&>(*this)));
  }
};

template<typename Derived, typename Base=Derived>
class ClonableAlignedImpl: virtual Clonable<Base> {
public:
  /// @return a deep copy of this object
  boost::shared_ptr<Base> clone() const override {
    return boost::shared_ptr<Base>(boost::allocate_shared<Derived>(
          Eigen::aligned_allocator<Derived>(),
          static_cast<const Derived&>(*this)));
  }
};

}
