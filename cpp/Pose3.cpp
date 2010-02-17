/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include <iostream>
#include "Pose3.h"
#include "Lie-inl.h"
#include "LieConfig.h"

using namespace std;
using namespace boost::numeric::ublas;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Pose3);

  /* ************************************************************************* */
  void Pose3::print(const string& s) const {
    R_.print(s + ".R");
    t_.print(s + ".t");
  }

  /* ************************************************************************* */
  bool Pose3::equals(const Pose3& pose, double tol) const {
    return R_.equals(pose.R_,tol) && t_.equals(pose.t_,tol);
  }

  /* ************************************************************************* */

#ifdef SLOW_BUT_CORRECT_EXPMAP

  /** Agrawal06iros versions of expmap and logmap*/
  template<> Pose3 expmap(const Vector& d) {
  	Vector w = vector_range<const Vector>(d, range(0,3));
  	Vector u = vector_range<const Vector>(d, range(3,6));
  	double t = norm_2(w);
		if (t < 1e-5)
			return Pose3(Rot3(), expmap<Point3> (u));
		else {
			Matrix W = skewSymmetric(w/t);
			Matrix A = eye(3, 3) + ((1 - cos(t)) / t) * W + ((t - sin(t)) / t) * (W * W);
			return Pose3(expmap<Rot3> (w), expmap<Point3> (A * u));
		}
  }

  Vector logmap(const Pose3& p) {
    Vector w = logmap(p.rotation()), T = p.translation().vector();
  	double t = norm_2(w);
		if (t < 1e-5)
	    return concatVectors(2, &w, &T);
		else {
			Matrix W = skewSymmetric(w/t);
			Matrix Ainv = eye(3, 3) - 0.5*t* W + ((2*sin(t)-t*(1+cos(t)))/2*sin(t)) * (W * W);
			Vector u = Ainv*T;
	    return concatVectors(2, &w, &u);
		}
  }

#else

  /* incorrect versions for which we know how to compute derivatives */
  template<> Pose3 expmap(const Vector& d) {
    Vector w = sub(d, 0,3);
    Vector u = sub(d, 3,6);
    return Pose3(expmap<Rot3> (w), expmap<Point3> (u));
  }

  // Log map at identity - return the translation and canonical rotation
  // coordinates of a pose.
  Vector logmap(const Pose3& p) {
    const Vector w = logmap(p.rotation()), u = logmap(p.translation());
    return concatVectors(2, &w, &u);
  }

#endif

  /* ************************************************************************* */
  Matrix Pose3::matrix() const {
    const Matrix R = R_.matrix(), T = Matrix_(3,1, t_.vector());
    const Matrix A34 = collect(2, &R, &T);
    const Matrix A14 = Matrix_(1,4, 0.0, 0.0, 0.0, 1.0);
    return gtsam::stack(2, &A34, &A14);
  }

  /* ************************************************************************* */
  Pose3 Pose3::transform_to(const Pose3& pose) const {
		Rot3 cRv = R_ * Rot3(inverse(pose.R_));
		Point3 t = gtsam::transform_to(pose, t_);
		return Pose3(cRv, t);
	}

  /* ************************************************************************* */
  Point3 transform_from(const Pose3& pose, const Point3& p) {
    return pose.rotation() * p + pose.translation();
  }

  /* ************************************************************************* */
  Matrix Dtransform_from1(const Pose3& pose, const Point3& p) {
#ifdef NEW_EXMAP
    Point3 q = transform_from(pose,p);
    Matrix DR = skewSymmetric(-q.x(), -q.y(), -q.z());
#else
    Matrix DR = Drotate1(pose.rotation(), p);
#endif
    Matrix Dt = eye(3);
    return collect(2,&DR,&Dt);
  }

  /* ************************************************************************* */
  Matrix Dtransform_from2(const Pose3& pose) {
    return pose.rotation().matrix();
  }

  /* ************************************************************************* */
  Point3 transform_to(const Pose3& pose, const Point3& p) {
    Point3 sub = p - pose.translation();
    return unrotate(pose.rotation(), sub);
  }

  /* ************************************************************************* */
  Matrix Dtransform_to1(const Pose3& pose, const Point3& p) {
    Point3 q = transform_to(pose,p);
    Matrix DR = skewSymmetric(q.x(), q.y(), q.z());
    Matrix DT = - pose.rotation().transpose(); // negative because of sub
    return collect(2,&DR,&DT);
  }

  /* ************************************************************************* */
  Matrix Dtransform_to2(const Pose3& pose, const Point3& p) {
    return pose.rotation().transpose();
  }

  /* ************************************************************************* */
  // compose = Pose3(compose(R1,R2),transform_from(p1,t2);

  Matrix Dcompose1(const Pose3& p1, const Pose3& p2) {
  	Matrix DR_R1 = p2.rotation().transpose();
		Matrix DR_t1 = zeros(3, 3);
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt = Dtransform_from1(p1, p2.translation());
		return gtsam::stack(2, &DR, &Dt);
	}

	Matrix Dcompose2(const Pose3& p1, const Pose3& p2) {
		Matrix R1 = p1.rotation().matrix();
		const static Matrix I = eye(3,3);
		const static Matrix Z3 = zeros(3, 3);
		Matrix DR = collect(2, &I, &Z3);
		Matrix Dt = collect(2, &Z3, &R1);
		return gtsam::stack(2, &DR, &Dt);
	}

  /* ************************************************************************* */
  // inverse = Pose3(inverse(R),-unrotate(R,t));
	Matrix Dinverse(const Pose3& p) {
		Matrix Rt = p.rotation().transpose();
		Matrix DR_R1 = -p.rotation().matrix();
		Matrix DR_t1 = zeros(3, 3);
		Matrix DR = collect(2, &DR_R1, &DR_t1);
		Matrix Dt_R1 = -skewSymmetric(unrotate(p.rotation(),p.translation()).vector());
		Matrix Dt_t1 = -Rt;
		Matrix Dt = collect(2, &Dt_R1, &Dt_t1);
		return gtsam::stack(2, &DR, &Dt);
	}

  /* ************************************************************************* */
  // between = compose(p2,inverse(p1));
  Pose3 between(const Pose3& p1, const Pose3& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) {
		Pose3 invp1 = inverse(p1);
		Pose3 result = compose(invp1, p2);
		if (H1) *H1 = Dcompose1(invp1, p2) * Dinverse(p1);
		if (H2) *H2 = Dcompose2(invp1, p2);
		return result;
	}

  /* ************************************************************************* */
} // namespace gtsam
