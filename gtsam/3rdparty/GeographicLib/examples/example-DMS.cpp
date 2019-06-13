// Example of using the GeographicLib::DMS class

#include <iostream>
#include <string>
#include <exception>
#include <GeographicLib/DMS.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    {
      string dms = "30d14'45.6\"S";
      DMS::flag type;
      double ang = DMS::Decode(dms, type);
      cout << type << " " << ang << "\n";
    }
    {
      double ang = -30.245715;
      string dms = DMS::Encode(ang, 6, DMS::LATITUDE);
      cout << dms << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
