/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include <gtsam/base/Lie-inl.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/concepts.h>
#include <iostream>
#include <cmath>

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Pose3);

  /** instantiate concept checks */
  GTSAM_CONCEPT_POSE_INST(Pose3);

  static const Matrix I3 = eye(3), Z3 = zeros(3, 3), _I3=-I3, I6 = eye(6);

  /* ************************************************************************* */
  // Calculate Adjoint map
  // Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
  // Experimental - unit tests of derivatives based on it do not check out yet
  Matrix Pose3::adjointMap() const {
		const Matrix R = R_.matrix();
		const Vector t = t_.vector();
		Matrix A = skewSymmetric(t)*R;
		Matrix DR = collect(2, &R, &Z3);
		Matrix Dt = collect(2, &A, &R);
		return gtsam::stack(2, &DR, &Dt);
	}

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
  /** Modified from Murray94book version (which assumes w and v normalized?) */
  Pose3 Pose3::Expmap(const Vector& xi) {

  	// get angular velocity omega and translational velocity v from twist xi
  	Point3 w(xi(0),xi(1),xi(2)), v(xi(3),xi(4),xi(5));

    double theta = w.norm();
		if (theta < 1e-10) {
		  static const Rot3 I;
			return Pose3(I, v);
		}
		else {
			Point3 n(w/theta); // axis unit vector
			Rot3 R = Rot3::rodriguez(n.vector(),theta);
			double vn = n.dot(v); // translation parallel to n
			Point3 n_cross_v = n.cross(v); // points towards axis
			Point3 t = (n_cross_v - R*n_cross_v)/theta + vn*n;
			return Pose3(R, t);
		}
  }

  /* ************************************************************************* */
  Vector Pose3::Logmap(const Pose3& p) {
    Vector w = Rot3::Logmap(p.rotation()), T = p.translation().vector();
  	double t = w.norm();
		if (t < 1e-10)
	    return concatVectors(2, &w, &T);
		else {
			Matrix W = skewSymmetric(w/t);
			// Formula from Agrawal06iros, equation (14)
			// simplified with Mathematica, and multiplying in T to avoid matrix math
			double Tan = tan(0.5*t);
			Vector WT = W*T;
			Vector u = T - (0.5*t)*WT + (1 - t/(2.*Tan)) * (W * WT);
	    return concatVectors(2, &w, &u);
		}
  }

  /* ************************************************************************* */
	// Different versions of retract
  Pose3 Pose3::retract(const Vector& xi, Pose3::CoordinatesMode mode) const {
    if(mode == Pose3::EXPMAP) {
      // Lie group exponential map, traces out geodesic
      return compose(Expmap(xi));
    } else if(mode == Pose3::FIRST_ORDER) {
      Vector omega(sub(xi, 0, 3));
      Point3 v(sub(xi, 3, 6));

      // R is always done exactly in all three retract versions below
      Rot3 R = R_.retract(omega);

      // Incorrect version
      // Retracts R and t independently
      // Point3 t = t_.retract(v.vector());

      // First order t approximation
      Point3 t = t_ + R_ * v;

      // Second order t approximation
      // Point3 t = t_ + R_ * (v+Point3(omega).cross(v)/2);

      return Pose3(R, t);
    } else {
      assert(false);
      exit(1);
    }
	}

  /* ************************************************************************* */
  // different versions of localCoordinates
	Vector Pose3::localCoordinates(const Pose3& T, Pose3::CoordinatesMode mode) const {
    if(mode == Pose3::EXPMAP) {
      // Lie group logarithm map, exact inverse of exponential map
      return Logmap(between(T));
    } else if(mode == Pose3::FIRST_ORDER) {
      // R is always done exactly in all three retract versions below
      Vector omega = R_.localCoordinates(T.rotation());

      // Incorrect version
      // Independently computes the logmap of the translation and rotation
      // Vector v = t_.localCoordinates(T.translation());

      // Correct first order t inverse
      Point3 d = R_.unrotate(T.translation() - t_);

      // TODO: correct second order t inverse

      return Vector_(6,omega(0),omega(1),omega(2),d.x(),d.y(),d.z());
    } else {
      assert(false);
      exit(1);
    }
	}

  /* ************************************************************************* */
  Matrix Pose3::matrix() const {
    const Matrix R = R_.matrix(), T = Matrix_(3,1, t_.vector());
    const Matrix A34 = collect(2, &R, &T);
    const Matrix A14 = Matrix_(1,4, 0.0, 0.0, 0.0, 1.0);
    return gtsam::stack(2, &A34, &A14);
  }

  /* ************************************************************************* */
  Pose3 Pose3::transform_to(const Pose3& pose) const {
		Rot3 cRv = R_ * Rot3(pose.R_.inverse());
		Point3 t = pose.transform_to(t_);
		return Pose3(cRv, t);
	}

  /* ************************************************************************* */
  Point3 Pose3::transform_from(const Point3& p,
		  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) {
			const Matrix R = R_.matrix();
			Matrix DR = R*skewSymmetric(-p.x(), -p.y(), -p.z());
			*H1 = collect(2,&DR,&R);
	  }
	  if (H2) *H2 = R_.matrix();
	  return R_ * p + t_;
  }

  /* ************************************************************************* */
  Point3 Pose3::transform_to(const Point3& p,
    		  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  const Point3 result = R_.unrotate(p - t_);
	  if (H1) {
			const Point3& q = result;
			Matrix DR = skewSymmetric(q.x(), q.y(), q.z());
			*H1 = collect(2, &DR, &_I3);
		}
	  if (H2) *H2 = R_.transpose();
	  return result;
  }

  /* ************************************************************************* */
  Pose3 Pose3::compose(const Pose3& p2,
		  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	  if (H1) *H1 = p2.inverse().adjointMap();
		if (H2) *H2 = I6;
		return (*this) * p2;
  }

  /* ************************************************************************* */
  Pose3 Pose3::inverse(boost::optional<Matrix&> H1) const {
  	if (H1) *H1 = -adjointMap();
  	Rot3 Rt = R_.inverse();
  	return Pose3(Rt, Rt*(-t_));
  }

  /* ************************************************************************* */
  // between = compose(p2,inverse(p1));
  Pose3 Pose3::between(const Pose3& p2, boost::optional<Matrix&> H1,
			boost::optional<Matrix&> H2) const {
	  	Matrix invH;
		Pose3 invp1 = inverse(invH);
		Matrix composeH1;
		Pose3 result = invp1.compose(p2, composeH1, H2);
		if (H1) *H1 = composeH1 * invH;
		return result;
  }

  /* ************************************************************************* */
  double Pose3::range(const Point3& point,
		  boost::optional<Matrix&> H1,
		  boost::optional<Matrix&> H2) const {
	  if (!H1 && !H2) return transform_to(point).norm();
	  Point3 d = transform_to(point, H1, H2);
	  double x = d.x(), y = d.y(), z = d.z(),
			 d2 = x * x + y * y + z * z, n = sqrt(d2);
	  Matrix D_result_d = Matrix_(1, 3, x / n, y / n, z / n);
	  if (H1) *H1 = D_result_d * (*H1);
	  if (H2) *H2 = D_result_d * (*H2);
	  return n;
  }

  /* ************************************************************************* */
  double Pose3::range(const Pose3& point,
  			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
  	 double r = range(point.translation(), H1, H2);
  	 if (H2) {
  		 Matrix H2_ = *H2 * point.rotation().matrix();
  		 *H2 = zeros(1, 6);
  		 insertSub(*H2, H2_, 0, 3);
  	 }
  	 return r;
  }
} // namespace gtsam
