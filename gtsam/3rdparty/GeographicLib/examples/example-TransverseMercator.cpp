// Example of using the GeographicLib::TransverseMercator class

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/TransverseMercator.hpp>

using namespace std;
using namespace GeographicLib;

// Define a UTM projection for an arbitrary ellipsoid
class UTMalt {
private:
  TransverseMercator _tm;       // The projection
  double _lon0;                 // Central longitude
  double _falseeasting, _falsenorthing;
public:
  UTMalt(double a,              // equatorial radius
         double f,              // flattening
         int zone,              // the UTM zone + hemisphere
         bool northp)
    : _tm(a, f, Constants::UTM_k0())
    , _lon0(6 * zone - 183)
    , _falseeasting(5e5)
    , _falsenorthing(northp ? 0 : 100e5) {
    if (!(zone >= 1 && zone <= 60))
      throw GeographicErr("zone not in [1,60]");
  }
  void Forward(double lat, double lon, double& x, double& y) {
    _tm.Forward(_lon0, lat, lon, x, y);
    x += _falseeasting;
    y += _falsenorthing;
  }
  void Reverse(double x, double y, double& lat, double& lon) {
    x -= _falseeasting;
    y -= _falsenorthing;
    _tm.Reverse(_lon0, x, y, lat, lon);
  }
};

int main() {
  try {
    UTMalt tm(6378388, 1/297.0, 30, true); // International ellipsoid, zone 30n
    {
      // Sample forward calculation
      double lat = 40.4, lon = -3.7; // Madrid
      double x, y;
      tm.Forward(lat, lon, x, y);
      cout << fixed << setprecision(0) << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      double x = 441e3, y = 4472e3;
      double lat, lon;
      tm.Reverse(x, y, lat, lon);
      cout << fixed << setprecision(5) << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
