// Example of using the GeographicLib::GeodesicLine class

#include <iostream>
#include <iomanip>
#include <exception>
#include <cmath>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Print waypoints between JFK and SIN
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geodesic& geod = Geodesic::WGS84();
    double
      lat1 = 40.640, lon1 = -73.779, // JFK
      lat2 =  1.359, lon2 = 103.989; // SIN
    GeodesicLine line = geod.InverseLine(lat1, lon1, lat2, lon2);
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
    {
      // Check that the inspection functions for exact = true return the same
      // as for GeodesicLineExact.
      Geodesic geoda(6.4e6, 0.5, true);
      GeodesicExact geodb(6.4e6, 0.5);
      GeodesicLine linea = geoda.InverseLine(lat1, lon1, lat2, lon2);
      GeodesicLineExact lineb = geodb.InverseLine(lat1, lon1, lat2, lon2);
      if (! (linea.Init() == lineb.Init() &&
             linea.Latitude() == lineb.Latitude() &&
             linea.Longitude() == lineb.Longitude() &&
             linea.Azimuth() == lineb.Azimuth() &&
             linea.EquatorialAzimuth() == lineb.EquatorialAzimuth() &&
             linea.EquatorialArc() == lineb.EquatorialArc() &&
             linea.EquatorialRadius() == lineb.EquatorialRadius() &&
             linea.Flattening() == lineb.Flattening() &&
             linea.Capabilities() == lineb.Capabilities() &&
             linea.Distance() == lineb.Distance() &&
             linea.Arc() == lineb.Arc()) )
        cerr << "Incompatible results compared to GeodesicLineExact\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
