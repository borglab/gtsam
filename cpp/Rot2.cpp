/*
 * Rot2.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Frank Dellaert
 */

#include "Rot2.h"

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  void Rot2::print(const string& s) const {
    cout << s << ":" << theta() << endl;
  }

  /* ************************************************************************* */
  bool Rot2::equals(const Rot2& R, double tol) const {
    return fabs(c_ - R.c_) <= tol && fabs(s_ - R.s_) <= tol;
  }

  /* ************************************************************************* */
  Rot2 Rot2::exmap(const Vector& v) const {
    if (zero(v)) return (*this);
    return Rot2(v(0)) * (*this); // exponential map then compose
  }

  /* ************************************************************************* */
  Matrix Rot2::matrix() const {
    return Matrix_(2, 2, c_, -s_, s_, c_);
  }

  /* ************************************************************************* */
  Matrix Rot2::transpose() const {
    return Matrix_(2, 2, c_, s_, -s_, c_);
  }

  /* ************************************************************************* */
  Matrix Rot2::negtranspose() const {
    return Matrix_(2, 2, -c_, -s_, s_, -c_);
  }

  /* ************************************************************************* */
  Rot2 Rot2::inverse() const { return Rot2(c_, -s_);}

  /* ************************************************************************* */
  Rot2 Rot2::invcompose(const Rot2& R) const {
    return Rot2(
        c_ * R.c_ + s_ * R.s_,
        -s_ * R.c_ + c_ * R.s_);
  }

  /* ************************************************************************* */
  Rot2 Rot2::operator*(const Rot2& R) const {
    return Rot2(
        c_ * R.c_ - s_ * R.s_,
        s_ * R.c_ + c_ * R.s_
    );
  }

  /* ************************************************************************* */
  Point2 Rot2::operator*(const Point2& p) const {
    return Point2(
        c_ * p.x() - s_ * p.y(),
        s_ * p.x() + c_ * p.y()
    );
  }

  /* ************************************************************************* */
  Point2 Rot2::unrotate(const Point2& p) const {
    return Point2(
        c_ * p.x() + s_ * p.y(),
        -s_ * p.x() + c_ * p.y()
    );
  }

  /* ************************************************************************* */
  Rot2 exmap(const Rot2& R, const Vector& v) {
    return R.exmap(v);
  }

  /* ************************************************************************* */
  Point2 rotate(const Rot2& R, const Point2& p) {
    return R * p;
  }

  /* ************************************************************************* */
  // see libraries/caml/geometry/math.ml
  Matrix Drotate1(const Rot2& R, const Point2& p) {
    Point2 q = R*p;
    return Matrix_(2, 1, -q.y(), q.x());
  }

  /* ************************************************************************* */
  Matrix Drotate2(const Rot2& R) {
    return R.matrix();
  }

  /* ************************************************************************* */
  Point2 unrotate(const Rot2& R, const Point2& p) {
    return R.unrotate(p);
  }

  /* ************************************************************************* */
  /** see libraries/caml/geometry/math.lyx, derivative of unrotate              */
  /* ************************************************************************* */
  Matrix Dunrotate1(const Rot2 & R, const Point2 & p) {
    Point2 q = R.unrotate(p);
    return Matrix_(2, 1, q.y(), -q.x());
  }

  /* ************************************************************************* */
  Matrix Dunrotate2(const Rot2 & R) {
    return R.transpose();
  }

} // gtsam
