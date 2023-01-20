/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleCamera.cpp
 * @brief A simple camera class with a Cal3_S2 calibration
 * @date June 30, 2012
 * @author Frank Dellaert
 */

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  SimpleCamera GTSAM_DEPRECATED simpleCamera(const Matrix34& P) {

    // P = [A|a] = s K cRw [I|-T], with s the unknown scale
    Matrix3 A = P.topLeftCorner(3, 3);
    Vector3 a = P.col(3);

    // do RQ decomposition to get s*K and cRw angles
    Matrix3 sK;
    Vector3 xyz;
    boost::tie(sK, xyz) = RQ(A);

    // Recover scale factor s and K
    double s = sK(2, 2);
    Matrix3 K = sK / s;

    // Recover cRw itself, and its inverse
    Rot3 cRw = Rot3::RzRyRx(xyz);
    Rot3 wRc = cRw.inverse();

    // Now, recover T from a = - s K cRw T = - A T
    Vector3 T = -(A.inverse() * a);
    return SimpleCamera(Pose3(wRc, T),
        Cal3_S2(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2)));
  }
#endif

}
