// Example of using the GeographicLib::Geohash class

#include <iostream>
#include <iomanip>
#include <exception>
#include <string>
#include <GeographicLib/Geohash.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    {
      // Sample forward calculation
      double lat = 57.64911, lon = 10.40744; // Jutland (the wikipedia example)
      string geohash;
      int maxlen = Geohash::GeohashLength(1.0e-5);
      for (int len = 0; len <= maxlen; ++len) {
        Geohash::Forward(lat, lon, len, geohash);
        cout << len << " " << geohash << "\n";
      }
    }
    {
      // Sample reverse calculation
      string geohash = "u4pruydqqvj";
      double lat, lon, latres, lonres;
      cout << fixed;
      for (unsigned i = 0; i <= geohash.length(); ++i) {
        int len;
        Geohash::Reverse(geohash.substr(0, i), lat, lon, len);
        latres = Geohash::LatitudeResolution(len);
        lonres = Geohash::LongitudeResolution(len);
        cout << setprecision(max(0, Geohash::DecimalPrecision(len)))
             << len << " "
             << lat << "+/-" << latres/2 << " "
             << lon << "+/-" << lonres/2 << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
