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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Lie-inl.h>
#include <iostream>
#include <cmath>

using namespace std;

namespace gtsam {

  /** Explicit instantiation of base class to export members */
  INSTANTIATE_LIE(Pose3);

  /** instantiate concept checks */
  GTSAM_CONCEPT_POSE_INST(Pose3);

  static const Matrix3 I3 = eye(3), Z3 = zeros(3, 3), _I3=-I3;
  static const Matrix6 I6 = eye(6);

  /* ************************************************************************* */
  Pose3::Pose3(const Pose2& pose2) :
			R_(Rot3::rodriguez(0, 0, pose2.theta())),
			t_(Point3(pose2.x(), pose2.y(), 0)) {
	}

  /* ************************************************************************* */
  // Calculate Adjoint map
  // Ad_pose is 6*6 matrix that when applied to twist xi, returns Ad_pose(xi)
  // Experimental - unit tests of derivatives based on it do not check out yet
  Matrix6 Pose3::adjointMap() const {
		const Matrix3 R = R_.matrix();
		const Vector3 t = t_.vector();
		Matrix3 A = skewSymmetric(t)*R;
		Matrix6 adj;
		adj << R, Z3, A, R;
		return adj;
	}

  /* ************************************************************************* */
  void Pose3::print(const string& s) const {
    cout << s;
    R_.print("R:\n");
    t_.print("t: ");
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
  Vector6 Pose3::Logmap(const Pose3& p) {
    Vector3 w = Rot3::Logmap(p.rotation()), T = p.translation().vector();
  	double t = w.norm();
		if (t < 1e-10) {
		  Vector6 log;
		  log << w, T;
		  return log;
		}
		else {
			Matrix3 W = skewSymmetric(w/t);
			// Formula from Agrawal06iros, equation (14)
			// simplified with Mathematica, and multiplying in T to avoid matrix math
			double Tan = tan(0.5*t);
			Vector3 WT = W*T;
			Vector3 u = T - (0.5*t)*WT + (1 - t/(2.*Tan)) * (W * WT);
			Vector6 log;
			log << w, u;
			return log;
		}
  }

  /* ************************************************************************* */
  Pose3 Pose3::retractFirstOrder(const Vector& xi) const {
      Vector3 omega(sub(xi, 0, 3));
      Point3 v(sub(xi, 3, 6));
      Rot3 R = R_.retract(omega);  // R is done exactly
      Point3 t = t_ + R_ * v; // First order t approximation
      return Pose3(R, t);
	}

  /* ************************************************************************* */
	// Different versions of retract
  Pose3 Pose3::retract(const Vector& xi, Pose3::CoordinatesMode mode) const {
    if(mode == Pose3::EXPMAP) {
    	// Lie group exponential map, traces out geodesic
      return compose(Expmap(xi));
    } else if(mode == Pose3::FIRST_ORDER) {
    	// First order
      return retractFirstOrder(xi);
    } else {
      // Point3 t = t_.retract(v.vector()); // Incorrect version retracts t independently
      // Point3 t = t_ + R_ * (v+Point3(omega).cross(v)/2); // Second order t approximation
      assert(false);
      exit(1);
    }
	}

  /* ************************************************************************* */
  // different versions of localCoordinates
	Vector6 Pose3::localCoordinates(const Pose3& T, Pose3::CoordinatesMode mode) const {
    if(mode == Pose3::EXPMAP) {
      // Lie group logarithm map, exact inverse of exponential map
      return Logmap(between(T));
    } else if(mode == Pose3::FIRST_ORDER) {
      // R is always done exactly in all three retract versions below
      Vector3 omega = R_.localCoordinates(T.rotation());

      // Incorrect version
      // Independently computes the logmap of the translation and rotation
      // Vector v = t_.localCoordinates(T.translation());

      // Correct first order t inverse
      Point3 d = R_.unrotate(T.translation() - t_);

      // TODO: correct second order t inverse
      Vector6 local;
      local << omega(0),omega(1),omega(2),d.x(),d.y(),d.z();
      return local;
    } else {
      assert(false);
      exit(1);
    }
	}

  /* ************************************************************************* */
  Matrix4 Pose3::matrix() const {
    const Matrix3 R = R_.matrix();
    const Vector3 T = t_.vector();
    Eigen::Matrix<double,1,4> A14;
    A14 << 0.0, 0.0, 0.0, 1.0;
    Matrix4 mat;
    mat << R, T, A14;
    return mat;
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
			H1->resize(3,6);
			(*H1) << DR, R;
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
			H1->resize(3,6);
			(*H1) << DR, _I3;
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
