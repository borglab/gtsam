// Example of using the GeographicLib::OSGB class

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/OSGB.hpp>
#include <GeographicLib/DMS.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    {
      // Sample forward calculation from
      // A guide to coordinate systems in Great Britain
      double
        lat = DMS::Decode(52,39,27.2531),
        lon = DMS::Decode( 1,43, 4.5177);
      double x, y;
      OSGB::Forward(lat, lon, x, y);
      string gridref;
      OSGB::GridReference(x, y, 2, gridref);
      cout << fixed << setprecision(3)
           << x << " " << y << " " << gridref << "\n";
    }
    {
      // Sample reverse calculation
      string gridref = "TG5113";
      double x, y;
      int prec;
      OSGB::GridReference(gridref, x, y, prec);
      double lat, lon;
      OSGB::Reverse(x, y, lat, lon);
      cout << fixed << setprecision(8)
           << prec << " " << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
