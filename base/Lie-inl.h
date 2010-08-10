/*
 * LieBaseImplementations.h
 *
 *  Created on: Jan 9, 2010
 *      Author: richard
 */

#pragma once

#include "Lie.h"

#define INSTANTIATE_LIE(T) \
  template T between(const T&, const T&); \
  template Vector logmap(const T&, const T&); \
  template T expmap(const T&, const Vector&); \
  template bool equal(const T&, const T&, double); \
  template bool equal(const T&, const T&); \
  template class Lie<T>;

namespace gtsam {
  template<class T>
  size_t Lie<T>::dim() const {
    return gtsam::dim(*((T*)this));
  }

  /**
   * Returns Exponential mapy
   * This is for matlab wrapper
   */
  template<class T>
  T Lie<T>::expmap(const Vector& v) const {
    return gtsam::expmap(*((T*)this),v);
  }

  /**
   * Returns Log map
   */
  template<class T>
  Vector Lie<T>::logmap(const T& lp) const {
    return gtsam::logmap(*((T*)this),lp);
  }

}
