// Example of using the GeographicLib::Geocentric class

#include <iostream>
#include <exception>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    {
      // Sample forward calculation
      double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
      double X, Y, Z;
      earth.Forward(lat, lon, h, X, Y, Z);
      cout << floor(X / 1000 + 0.5) << " "
           << floor(Y / 1000 + 0.5) << " "
           << floor(Z / 1000 + 0.5) << "\n";
    }
    {
      // Sample reverse calculation
      double X = 302e3, Y = 5636e3, Z = 2980e3;
      double lat, lon, h;
      earth.Reverse(X, Y, Z, lat, lon, h);
      cout << lat << " " << lon << " " << h << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
