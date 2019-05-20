// Example of using the GeographicLib::GARS class

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/GARS.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    {
      // Sample forward calculation
      double lat = 57.64911, lon = 10.40744; // Jutland
      string gars;
      for (int prec = 0; prec <= 2; ++prec) {
        GARS::Forward(lat, lon, prec, gars);
        cout << prec << " " << gars << "\n";
      }
    }
    {
      // Sample reverse calculation
      string gars = "381NH45";
      double lat, lon;
      cout << fixed;
      for (int len = 5; len <= int(gars.size()); ++len) {
        int prec;
        GARS::Reverse(gars.substr(0, len), lat, lon, prec);
        cout << prec << " " << lat << " " << lon << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
