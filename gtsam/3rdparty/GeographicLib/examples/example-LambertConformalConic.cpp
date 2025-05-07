// Example of using the GeographicLib::LambertConformalConic class

#include <iostream>
#include <exception>
#include <GeographicLib/LambertConformalConic.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Define the Pennsylvania South state coordinate system EPSG:3364
    // https://www.spatialreference.org/ref/epsg/3364/
    const double
      a = Constants::WGS84_a(),
      f = 1/298.257222101,                      // GRS80
      lat1 = 40 + 58/60.0, lat2 = 39 + 56/60.0, // standard parallels
      k1 = 1,                                   // scale
      lat0 = 39 + 20/60.0, lon0 =-77 - 45/60.0, // origin
      fe = 600000, fn = 0;                      // false easting and northing
    // Set up basic projection
    const LambertConformalConic PASouth(a, f, lat1, lat2, k1);
    double x0, y0;
    // Transform origin point
    PASouth.Forward(lon0, lat0, lon0, x0, y0);
    x0 -= fe; y0 -= fn;
    {
      // Sample conversion from geodetic to PASouth grid
      double lat = 39.95, lon = -75.17;    // Philadelphia
      double x, y;
      PASouth.Forward(lon0, lat, lon, x, y);
      x -= x0; y -= y0;
      cout << x << " " << y << "\n";
    }
    {
      // Sample conversion from PASouth grid to geodetic
      double x = 820e3, y = 72e3;
      double lat, lon;
      x += x0; y += y0;
      PASouth.Reverse(lon0, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
