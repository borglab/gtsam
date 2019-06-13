// Example of using the GeographicLib::TransverseMercatorExact class

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/TransverseMercatorExact.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    TransverseMercatorExact proj(Constants::WGS84_a(), Constants::WGS84_f(),
                                 Constants::UTM_k0());
    // Alternatively:
    // const TransverseMercatorExact& proj = TransverseMercatorExact::UTM();
    double lon0 = -75;          // Central meridian for UTM zone 18
    {
      // Sample forward calculation
      double lat = 40.3, lon = -74.7; // Princeton, NJ
      double x, y;
      proj.Forward(lon0, lat, lon, x, y);
      cout << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      double x = 25e3, y = 4461e3;
      double lat, lon;
      proj.Reverse(lon0, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
