/**
 * @file   PhysicalConstants.h
 * @brief  House values that are repeatedly used
 * @author Ryan Watson
 */

#pragma once

namespace gtsam {

const int speedOfLight = 299792458;   // [m/s]
const int gravity = 9.80665;   // [m/s*s]
const double L1 = 1575.42e+6;   // GPS L1 freq [HZ]
const double L2 = 1227.6e+6;   // GPS L2 freq [Hz]
const double L5 = 1176.45e+6;   // GPS L5 freq [Hz]
const double semiMajor = 6378137.0;  // Earth's Semi-Major Axis [m]
const double semiMinor = 6356752.3142;    // Earth's Semi-Minor Axis [m]
const double earthRot = 7.292115e-5;     // Earth's rotation rate [rad/sec]
const int stdPressure = 1013;   // Std Atmosphere Pressure [mbar]
const double stdTemp = 288.15;    // Std Atmosphere Temp. [Kelvin]

}
