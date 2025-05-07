// Example of using the GeographicLib::Intersect class

#include <iostream>
#include <exception>
#include <GeographicLib/Intersect.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    Intersect intersect(geod);
    // Define the two geodesics
    GeodesicLine lineX(geod, 0, 0, 45, Intersect::LineCaps);
    GeodesicLine lineY(geod, 45, 10, 135, Intersect::LineCaps);
    // Find displacement to closest intersection
    Intersect::Point point = intersect.Closest(lineX, lineY);
    // Check position at intersection
    double latx, lonx, laty, lony;
    lineX.Position(point.first, latx, lonx);
    lineY.Position(point.second, laty, lony);
    cout << "X intersection displacement + position "
         << point.first << " " << latx << " " << lonx << "\n";
    cout << "Y intersection displacement + position "
         << point.second << " " << laty << " " << lony << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
