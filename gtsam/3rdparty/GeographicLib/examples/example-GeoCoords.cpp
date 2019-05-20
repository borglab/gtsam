// Example of using the GeographicLib::GeoCoords class

#include <iostream>
#include <exception>
#include <GeographicLib/GeoCoords.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Miscellaneous conversions
    double lat = 33.3, lon = 44.4;
    GeoCoords c(lat, lon);
    cout << c.MGRSRepresentation(-3) << "\n";
    c.Reset("18TWN0050");
    cout << c.DMSRepresentation() << "\n";
    cout << c.Latitude() << " " << c.Longitude() << "\n";
    c.Reset("1d38'W 55d30'N");
    cout << c.GeoRepresentation() << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
