/**
 *  @file   gnssTestHelpers.h
 *  @brief  Shared test fixtures for GNSS factor unit tests.
 *
 *  Header-only utilities and named constants reused across the pseudorange
 *  and carrier-phase factor tests so the same satellite/base/observation
 *  values do not need to be repeated test by test.
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GnssCommon.h>

#include <cmath>

namespace gtsam {
namespace gnss_test {

/// GPS L1 wavelength (~0.1903 m).
inline constexpr double kLambdaL1 = 0.190293672798365;

/// Speed of light in vacuum (m/s) -- matches gtsam::gnss::C_LIGHT.
inline constexpr double kCLight = 299792458.0;

/// Build an ECEF-from-ENU transform from a geodetic origin (WGS84).
inline Pose3 makeEcefTnav(double lat_deg, double lon_deg, double h) {
  constexpr double deg2rad = M_PI / 180.0;
  const double lat = lat_deg * deg2rad;
  const double lon = lon_deg * deg2rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);

  Matrix3 R;
  R.col(0) = Vector3(-slon, clon, 0.0);
  R.col(1) = Vector3(-slat * clon, -slat * slon, clat);
  R.col(2) = Vector3(clat * clon, clat * slon, slat);

  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  const double e2 = 2.0 * f - f * f;
  const double N = a / std::sqrt(1.0 - e2 * slat * slat);
  const Point3 t((N + h) * clat * clon,
                 (N + h) * clat * slon,
                 (N * (1.0 - e2) + h) * slat);
  return Pose3(Rot3(R), t);
}

/// Representative ENU origin (Tokyo) used by ecef_T_nav tests.
namespace tokyo {
inline constexpr double kLatDeg = 35.6852;
inline constexpr double kLonDeg = 139.7528;
inline constexpr double kHeightM = 5.0;
inline Pose3 ecefTnav() {
  return makeEcefTnav(kLatDeg, kLonDeg, kHeightM);
}
}  // namespace tokyo

/// Sagnac-corrected geometric range (no Jacobian, for tests that only need
/// the scalar distance).
inline double geodistRange(const Point3& sat, const Point3& rcv) {
  Point3 e;
  return gnss::geodist(sat, rcv, e);
}

/// Shared geometry for double-difference factor tests.  Same numeric values
/// for both pseudorange and carrier-phase DD test fixtures.
namespace dd {
inline const Point3 kBasePos(-3961908.12, 3348995.59, 3698211.13);
inline const Point3 kSatRefRov(-5824269.46, -22935011.27, -12195522.22);
inline const Point3 kSatTargetRov(15524471.18, -6304441.44, 20851474.88);
// Satellite positions at base time (slightly different due to satellite motion).
inline const Point3 kSatRefBase(-5824242.10, -22935002.50, -12195510.80);
inline const Point3 kSatTargetBase(15524505.30, -6304460.20, 20851440.60);

/// "True" rover ECEF position used by zero-error and Jacobian tests.
inline const Point3 kTruePos(-3961900.00, 3349000.00, 3698215.00);
}  // namespace dd

/// Realistic single-satellite measurement borrowed from
/// `SinglePointPositioningExample.ipynb`, reused by several Jacobian tests.
namespace sample {
inline const Point3 kSatPos(-5824269.46342, -22935011.26952, -12195522.22428);
inline const Point3 kReceiverPos(-3961908.12, 3348995.59, 3698211.13);
inline constexpr double kSatClkBias = -0.00022743876852667193;
inline constexpr double kReceiverClock = 5.377885093511699e-07;
inline constexpr double kPseudorange = 24874028.989;
}  // namespace sample

}  // namespace gnss_test
}  // namespace gtsam
