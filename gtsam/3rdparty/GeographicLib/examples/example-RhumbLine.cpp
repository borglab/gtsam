// Example of using the GeographicLib::RhumbLine class

#include <iostream>
#include <iomanip>
#include <exception>
#include <cmath>
#include <GeographicLib/Rhumb.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Print waypoints between JFK and SIN
    Rhumb rhumb(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Rhumb& rhumb = Rhumb::WGS84();
    double
      lat1 = 40.640, lon1 = -73.779, // JFK
      lat2 =  1.359, lon2 = 103.989; // SIN
    double s12, azi12;
    rhumb.Inverse(lat1, lon1, lat2, lon2, s12, azi12);
    const GeographicLib::RhumbLine line = rhumb.Line(lat1, lon1, azi12);
    // Alternatively
    // const GeographicLib::RhumbLine line = rhumb.Line(lat1, lon1, azi1);
    double ds0 = 500e3;             // Nominal distance between points = 500 km
    int num = int(ceil(s12 / ds0)); // The number of intervals
    cout << fixed << setprecision(3);
    {
      // Use intervals of equal length
      double ds = s12 / num;
      for (int i = 0; i <= num; ++i) {
        double lat, lon;
       line.Position(i * ds, lat, lon);
       cout << i << " " << lat << " " << lon << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
