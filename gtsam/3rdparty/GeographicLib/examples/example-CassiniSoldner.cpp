// Example of using the GeographicLib::CassiniSoldner class

#include <iostream>
#include <exception>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/CassiniSoldner.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geodesic& geod = Geodesic::WGS84();
    const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
    CassiniSoldner proj(lat0, lon0, geod);
    {
      // Sample forward calculation
      double lat = 50.9, lon = 1.8; // Calais
      double x, y;
      proj.Forward(lat, lon, x, y);
      cout << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      double x = -38e3, y = 230e3;
      double lat, lon;
      proj.Reverse(x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
