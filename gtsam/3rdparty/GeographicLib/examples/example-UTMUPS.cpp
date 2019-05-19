// Example of using the GeographicLib::UTMUPS class

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/UTMUPS.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // See also example-GeoCoords.cpp
    {
      // Sample forward calculation
      double lat = 33.3, lon = 44.4; // Baghdad
      int zone;
      bool northp;
      double x, y;
      UTMUPS::Forward(lat, lon, zone, northp, x, y);
      string zonestr = UTMUPS::EncodeZone(zone, northp);
      cout << fixed << setprecision(2)
           << zonestr << " " << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      string zonestr = "38n";
      int zone;
      bool northp;
      UTMUPS::DecodeZone(zonestr, zone, northp);
      double x = 444e3, y = 3688e3;
      double lat, lon;
      UTMUPS::Reverse(zone, northp, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
