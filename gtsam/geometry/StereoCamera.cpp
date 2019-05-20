/* ----------------------------------------------------------------------------

*GTSAM Copyright 2010, Georgia Tech Research Corporation,
*Atlanta, Georgia 30332-0415
*All Rights Reserved
*Authors: Frank Dellaert, et al. (see THANKS for the full author list)

*See LICENSE for the license information

*-------------------------------------------------------------------------- */

/**
*@file    StereoCamera.h
*@brief   A Stereo Camera based on two Simple Cameras
*@author  Chris Beall
 */

#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

  /* ************************************************************************* */
  StereoCamera::StereoCamera(const Pose3& leftCamPose,
      const Cal3_S2Stereo::shared_ptr K) :
      leftCamPose_(leftCamPose), K_(K) {
  }

  /* ************************************************************************* */
  StereoPoint2 StereoCamera::project(const Point3& point) const {
    return project2(point);
  }

  /* ************************************************************************* */
  StereoPoint2 StereoCamera::project2(const Point3& point,
      OptionalJacobian<3,6> H1, OptionalJacobian<3,3> H2) const {

    const Point3 q = leftCamPose_.transformTo(point);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    if (q.z() <= 0)
      throw StereoCheiralityException();
#endif

    // get calibration
    const Cal3_S2Stereo& K = *K_;
    const double fx = K.fx(), fy = K.fy(), b = K.baseline();

    // calculate scaled but not translated image coordinates
    const double d = 1.0 / q.z();
    const double x = q.x(), y = q.y();
    const double dfx = d*fx, dfy = d*fy;
    const double uL = dfx*x;
    const double uR = dfx*(x - b);
    const double v  = dfy*y;

    // check if derivatives need to be computed
    if (H1 || H2) {
      // optimized version, see StereoCamera.nb
      if (H1) {
        const double v1 = v/fy, v2 = fx*v1, dx=d*x;
        *H1  << uL*v1, -fx-dx*uL,     v2, -dfx,  0.0, d*uL,
                uR*v1, -fx-dx*uR,     v2, -dfx,  0.0, d*uR,
                fy + v*v1,    -dx*v , -x*dfy,  0.0, -dfy, d*v;
      }
      if (H2) {
        const Matrix3 R(leftCamPose_.rotation().matrix());
        *H2  << fx*R(0, 0) - R(0, 2)*uL, fx*R(1, 0) - R(1, 2)*uL, fx*R(2, 0) - R(2, 2)*uL,
                fx*R(0, 0) - R(0, 2)*uR, fx*R(1, 0) - R(1, 2)*uR, fx*R(2, 0) - R(2, 2)*uR,
                fy*R(0, 1) - R(0, 2)*v , fy*R(1, 1) - R(1, 2)*v , fy*R(2, 1) - R(2, 2)*v;
        *H2 << d * (*H2);
      }
    }

    // finally translate
    return StereoPoint2(K.px() + uL, K.px() + uR, K.py() + v);
  }

  /* ************************************************************************* */
  StereoPoint2 StereoCamera::project(const Point3& point,
      OptionalJacobian<3,6> H1, OptionalJacobian<3,3> H2,
      OptionalJacobian<3,0> H3) const {
    if (H3)
    throw std::runtime_error(
        "StereoCamera::project does not support third derivative - BTW use project2");
    return project2(point,H1,H2);
  }

  /* ************************************************************************* */
  Point3 StereoCamera::backproject(const StereoPoint2& z) const {
    Vector measured = z.vector();
    double Z = K_->baseline() * K_->fx() / (measured[0] - measured[1]);
    double X = Z * (measured[0] - K_->px()) / K_->fx();
    double Y = Z * (measured[2] - K_->py()) / K_->fy();
    Point3 point = leftCamPose_.transformFrom(Point3(X, Y, Z));
    return point;
  }

  /* ************************************************************************* */
  Point3 StereoCamera::backproject2(const StereoPoint2& z, OptionalJacobian<3, 6> H1,
                                    OptionalJacobian<3, 3> H2)  const {
    const Cal3_S2Stereo& K = *K_;
    const double fx = K.fx(), fy = K.fy(), cx = K.px(), cy = K.py(), b = K.baseline();

    double uL = z.uL(), uR = z.uR(), v = z.v();
    double disparity = uL - uR;

    double local_z = b * fx / disparity;
    const Point3 local(local_z * (uL - cx)/ fx, local_z * (v - cy) / fy, local_z);

    if(H1 || H2) {
      double z_partial_uR = local_z/disparity;
      double x_partial_uR = local.x()/disparity;
      double y_partial_uR = local.y()/disparity;
      Matrix3 D_local_z;
      D_local_z << -x_partial_uR + local.z()/fx, x_partial_uR, 0,
          -y_partial_uR, y_partial_uR, local.z() / fy,
          -z_partial_uR, z_partial_uR, 0;

      Matrix3 D_point_local;
      const Point3 point = leftCamPose_.transformFrom(local, H1, D_point_local);

      if(H2) {
        *H2 = D_point_local * D_local_z;
      }

      return point;
    }

    return leftCamPose_.transformFrom(local);
  }

}
