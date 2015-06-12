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

    const Point3 q = leftCamPose_.transform_to(point);

    if ( q.z() <= 0 ) throw StereoCheiralityException();

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
    Point3 world_point = leftCamPose_.transform_from(Point3(X, Y, Z));
    return world_point;
  }

  /* ************************************************************************* */
  Point3 StereoCamera::backproject2(const StereoPoint2& z, OptionalJacobian<3, 6> H1,
                                    OptionalJacobian<3, 3> H2)  const {
    const Cal3_S2Stereo& K = *K_;
    const double fx = K.fx(), fy = K.fy(), cx = K.px(), cy = K.py(), b = K.baseline();

    Vector3 measured = z.vector();   // u_L, u_R, v
    double d = measured[0] - measured[1]; // disparity

    double Z = b * fx / (measured[0] - measured[1]);
    double X = Z * (measured[0] - cx) / fx;
    double Y = Z * (measured[2] - cy) / fy;

    if(H1 || H2) {
      if(H1) {

      }
      if(H2) {
        double d_2 = d*d;
        double z_partial_x = -fx*b/d_2, z_partial_y = fx*b/d_2;
        *H2 << z_partial_x * X/Z + Z/fx, z_partial_y *X/Z, 0,
            z_partial_x * Y/Z, z_partial_y *Y/Z, Z/fy,
            z_partial_x, z_partial_y, 0;
      }

      Matrix point_H1, point_H2;
      const Point3 point = leftCamPose_.transform_from(Point3(X,Y,Z), point_H1, point_H2);

      *H1 = point_H1 * (*H1);
      *H2 = point_H2 * (*H2);

      return point;
    }

    return leftCamPose_.transform_from(Point3(X, Y, Z));
  }

}
