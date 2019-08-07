// Example of using the GeographicLib::GARS class

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/Georef.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    {
      // Sample forward calculation
      double lat = 57.64911, lon = 10.40744; // Jutland
      string georef;
      for (int prec = -1; prec <= 11; ++prec) {
        Georef::Forward(lat, lon, prec, georef);
        cout << prec << " " << georef << "\n";
      }
    }
    {
      // Sample reverse calculation
      string georef = "NKLN2444638946";
      double lat, lon;
      int prec;
      cout << fixed;
      Georef::Reverse(georef.substr(0, 2), lat, lon, prec);
      cout << prec << " " << lat << " " << lon << "\n";
      Georef::Reverse(georef.substr(0, 4), lat, lon, prec);
      cout << prec << " " << lat << " " << lon << "\n";
      Georef::Reverse(georef, lat, lon, prec);
      cout << prec << " " << lat << " " << lon << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
