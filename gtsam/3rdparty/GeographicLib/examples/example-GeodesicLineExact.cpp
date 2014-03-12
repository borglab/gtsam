// Example of using the GeographicLib::GeodesicLineExact class

#include <iostream>
#include <exception>
#include <cmath>
#include <iomanip>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/GeodesicLineExact.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Print waypoints between JFK and SIN
    GeodesicExact geod(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const GeodesicExact& geod = GeodesicExact::WGS84;
    double
      lat1 = 40.640, lon1 = -73.779, // JFK
      lat2 =  1.359, lon2 = 103.989; // SIN
    double s12, azi1, azi2,
      a12 = geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
    const GeographicLib::GeodesicLineExact line(geod, lat1, lon1, azi1);
    // Alternatively
    // const GeographicLib::GeodesicLineExact line = geod.Line(lat1, lon1, azi1);
    double ds = 500e3;          // Nominal distance between points = 500 km
    int num = int(ceil(s12 / ds)); // The number of intervals
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
    {
      // Slightly faster, use intervals of equal arc length
      double da = a12 / num;
      for (int i = 0; i <= num; ++i) {
        double lat, lon;
       line.ArcPosition(i * da, lat, lon);
       cout << i << " " << lat << " " << lon << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
