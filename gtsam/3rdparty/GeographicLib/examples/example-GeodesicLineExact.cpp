// Example of using the GeographicLib::GeodesicLineExact class

#include <iostream>
#include <iomanip>
#include <exception>
#include <cmath>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/GeodesicLineExact.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Print waypoints between JFK and SIN
    GeodesicExact geod(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const GeodesicExact& geod = GeodesicExact::WGS84();
    double
      lat1 = 40.640, lon1 = -73.779, // JFK
      lat2 =  1.359, lon2 = 103.989; // SIN
    const GeographicLib::GeodesicLineExact line =
      geod.InverseLine(lat1, lon1, lat2, lon2);
    double ds0 = 500e3;             // Nominal distance between points = 500 km
    int num = int(ceil(line.Distance() / ds0)); // The number of intervals
    cout << fixed << setprecision(3);
    {
      // Use intervals of equal length
      double ds = line.Distance() / num;
      for (int i = 0; i <= num; ++i) {
        double lat, lon;
       line.Position(i * ds, lat, lon);
       cout << i << " " << lat << " " << lon << "\n";
      }
    }
    {
      // Slightly faster, use intervals of equal arc length
      double da = line.Arc() / num;
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
}
