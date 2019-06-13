#include "cgeoid.h"
#include "GeographicLib/Geoid.hpp"

extern "C"
double HeightAboveEllipsoid(double lat, double lon, double h) {
  try {
    // Declare static so that g is only constructed once
    static const GeographicLib::Geoid g("egm2008-1");
    return h + GeographicLib::Geoid::GEOIDTOELLIPSOID * g(lat, lon);
  }
  catch (...) {
    return GeographicLib::Math::NaN();
  }
}
