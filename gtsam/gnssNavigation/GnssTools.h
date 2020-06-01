/**
 * @file   GnssTools.h
 * @brief  Tools required to process GNSS data -- (i.e. ECEF to ENU transformation)
 * @author Ryan Watson & Jason Gross
 */

#pragma once

#include <gtsam/config.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/VectorSpace.h>
#include <boost/serialization/nvp.hpp>
#include <gtsam/gnssNavigation/PhysicalConstants.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

namespace gtsam {

//// rotate from ECI to ECEF
Point3 inertialToECEF( const Point3& inertialPosition, const double t, const double t0);

//// Generate rotation matrix from Earth-to-Navigation frame
Matrix earthToNavTrans( const Point3& ECEFxyz);

//// Compute mapping from meas. to states
Vector obsMap(const Point3& p1, const Point3& q, const int& Trop = 1);

//// Compute mapping from meas. to states
Vector obsMapNED(const Point3& p1, const Point3& q, const int& Trop = 1);


//// Extract PRN Vector from GNSS data structure
Eigen::VectorXi getPRN(const Matrix& p);

//// See if current PRN value was present at previous epoch
bool checkPRN(const Eigen::VectorXi& p, const int& n);

/// computer delta pseudorange observables
double deltaObs(const Point3& p1, const Point3& p2, const double& pseudorange);

/// compute the delta troposphere correction
double deltaTrop(const Point3& p1, const Point3& p2);

//// Convert from WGS-84 ECEF coordinated to local-level-tangent (ENU) coordinates
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
Point3 xyz2enu(const Point3& p1, const Point3& p2);

//// Convert WGS-84 ECEF coordinates to LLH
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
Point3 xyz2llh(const Point3& p1);

//// Convert ENU coordinates to ECEF
////
//// REF :: Groves, Paul. Principles of GNSS, Inertial, and Multisensor Integrated
////        Navigation Systems. Artech House, 2008
Point3 enu2xyz(const Point3& p1, const Point3& p2);

/// Convert NED Local Frame to ENU Local Frame
Point3 ned2enu(const Point3& p1);

//// Computer Elevation of Satellite from Receiver
double calcEl(const Point3& p1, const Point3& p2);

//// Compute elevation angle given a NED position vector.
double calcElNed(const Point3& p1);

//// Computer elevation angle dependant weighting.
double elDepWeight(const Point3& p1, const Point3& p2, double measWeight);

//// Elevation angle only troposphere mapping
////
//// REF :: Black, H. and Eisner, A., 1984. Correcting satellite Doppler data for
////        tropospheric effects. Journal of Geophysical Research.
double tropMap(const double& El);

//// Troposphere Model --- dry component only. Uses the Sass. model.
////
//// REF :: Saastamoinen, J. 1972, Atmospheric correction for the troposphere and
////        stratosphere in radio ranging of satellites, in The Use of Artificial
////        Satellites for Geodesy
double tropDry(const Point3& p1);

//// time difference carrier-phase observations
double dopplerObs(const Point3& p1, double tdcp1, const Point3& p2, double tdcp2);

}
