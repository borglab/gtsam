// Example of using the GeographicLib::Gnomonic class

#include <iostream>
#include <exception>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Gnomonic.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geodesic& geod = Geodesic::WGS84();
    const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
    Gnomonic proj(geod);
    {
      // Sample forward calculation
      double lat = 50.9, lon = 1.8; // Calais
      double x, y;
      proj.Forward(lat0, lon0, lat, lon, x, y);
      cout << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      double x = -38e3, y = 230e3;
      double lat, lon;
      proj.Reverse(lat0, lon0, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
