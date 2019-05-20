// Example of using the GeographicLib::PolarStereographic class

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/PolarStereographic.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    PolarStereographic proj(Constants::WGS84_a(), Constants::WGS84_f(),
                            Constants::UPS_k0());
    // Alternatively:
    // const PolarStereographic& proj = PolarStereographic::UPS();
    bool northp = true;
    {
      // Sample forward calculation
      double lat = 61.2, lon = -149.9; // Anchorage
      double x, y;
      proj.Forward(northp, lat, lon, x, y);
      cout << x << " " << y << "\n";
    }
    {
      // Sample reverse calculation
      double x = -1637e3, y = 2824e3;
      double lat, lon;
      proj.Reverse(northp, x, y, lat, lon);
      cout << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
