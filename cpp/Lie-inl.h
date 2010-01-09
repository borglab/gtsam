/*
 * LieBaseImplementations.h
 *
 *  Created on: Jan 9, 2010
 *      Author: richard
 */

#include "Lie.h"

namespace gtsam {
  template<class T>
  size_t Lie<T>::dim() const {
    return gtsam::dim(*((T*)this));
  }

  /**
   * Returns Exponential mapy
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
