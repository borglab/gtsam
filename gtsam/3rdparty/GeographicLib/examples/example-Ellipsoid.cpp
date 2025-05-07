// Example of using the GeographicLib::Ellipsoid class

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/Ellipsoid.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Ellipsoid wgs84(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Ellipsoid& wgs84 = Ellipsoid::WGS84();
    cout << "The latitude half way between the equator and the pole is "
         << wgs84.InverseRectifyingLatitude(45) << "\n";
    cout << "Half the area of the ellipsoid lies between latitudes +/- "
         << wgs84.InverseAuthalicLatitude(30) << "\n";
    cout << "The northernmost edge of a square Mercator map is at latitude "
         << wgs84.InverseIsometricLatitude(180) << "\n";
    cout << "Table phi(deg)  beta-phi xi-phi mu-phi chi-phi theta-phi (mins)\n"
         << fixed << setprecision(2);
    for (int i = 0; i <= 90; i += 15) {
      double phi = i,
        bet = wgs84.ParametricLatitude(phi),
        xi = wgs84.AuthalicLatitude(phi),
        mu = wgs84.RectifyingLatitude(phi),
        chi = wgs84.ConformalLatitude(phi),
        theta = wgs84.GeocentricLatitude(phi);
      cout << i << " "
           << (bet-phi)*60 << " "
           << (xi-phi)*60 << " "
           << (mu-phi)*60 << " "
           << (chi-phi)*60 << " "
           << (theta-phi)*60 << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
