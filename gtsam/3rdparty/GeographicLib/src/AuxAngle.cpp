/**
 * \file AuxAngle.cpp
 * \brief Implementation for the GeographicLib::AuxAngle class.
 *
 * This file is an implementation of the methods described in
 * - C. F. F. Karney,
 *   <a href="https://doi.org/10.1080/00396265.2023.2217604">
 *   On auxiliary latitudes,</a>
 *   Survey Review 56(395), 165--180 (2024);
 *   preprint
 *   <a href="https://arxiv.org/abs/2212.05818">arXiv:2212.05818</a>.
 * .
 * Copyright (c) Charles Karney (2022-2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/AuxAngle.hpp>

namespace GeographicLib {

  using namespace std;

  AuxAngle AuxAngle::NaN() {
    return AuxAngle(Math::NaN(), Math::NaN());
  }

  AuxAngle AuxAngle::normalized() const {
    using std::isnan;           // Needed for Centos 7, ubuntu 14
    if ( isnan( tan() ) ||
         (fabs(_y) > numeric_limits<real>::max()/2 &&
          fabs(_x) > numeric_limits<real>::max()/2) )
      // deal with
      // (0,0), (inf,inf), (nan,nan), (nan,x), (y,nan), (toobig,toobig)
      return NaN();
    real r = hypot(_y, _x),
      y = _y/r, x = _x/r;
    // deal with r = inf, then one of y,x becomes 1
    if (isnan(y)) y = copysign(real(1), _y);
    if (isnan(x)) x = copysign(real(1), _x);
    return AuxAngle(y, x);
  }

  AuxAngle AuxAngle::copyquadrant(const AuxAngle& p) const {
    return AuxAngle(copysign(y(), p.y()), copysign(x(), p.x()));
  }

  AuxAngle& AuxAngle::operator+=(const AuxAngle& p) {
    // Do nothing if p.tan() == 0 to preserve signs of y() and x()
    if (p.tan() != 0) {
      real x = _x * p._x - _y * p._y;
      _y = _y * p._x + _x * p._y;
      _x = x;
    }
    return *this;
  }

} // namespace GeographicLib
