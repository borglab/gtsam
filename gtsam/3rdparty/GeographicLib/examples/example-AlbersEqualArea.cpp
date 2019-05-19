// Example of using the GeographicLib::AlbersEqualArea class

#include <iostream>
#include <exception>
#include <GeographicLib/AlbersEqualArea.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
   const double
     a = Constants::WGS84_a(),
     f = Constants::WGS84_f(),
     lat1 = 40 + 58/60.0, lat2 = 39 + 56/60.0, // standard parallels
     k1 = 1,                                   // scale
     lon0 = -77 - 45/60.0;                     // Central meridian
   // Set up basic projection
   const AlbersEqualArea albers(a, f, lat1, lat2, k1);
   {
     // Sample conversion from geodetic to Albers Equal Area
     double lat = 39.95, lon = -75.17;    // Philadelphia
     double x, y;
     albers.Forward(lon0, lat, lon, x, y);
     cout << x << " " << y << "\n";
   }
   {
     // Sample conversion from Albers Equal Area grid to geodetic
     double x = 220e3, y = -53e3;
     double lat, lon;
     albers.Reverse(lon0, x, y, lat, lon);
     cout << lat << " " << lon << "\n";
   }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
