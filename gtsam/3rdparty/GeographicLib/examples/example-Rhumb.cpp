// Example of using the GeographicLib::Rhumb class

#include <iostream>
#include <exception>
#include <GeographicLib/Rhumb.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Rhumb rhumb(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Rhumb& rhumb = Rhumb::WGS84();
    {
      // Sample direct calculation, travelling about NE from JFK
      double lat1 = 40.6, lon1 = -73.8, s12 = 5.5e6, azi12 = 51;
      double lat2, lon2;
      rhumb.Direct(lat1, lon1, azi12, s12, lat2, lon2);
      cout << lat2 << " " << lon2 << "\n";
    }
    {
      // Sample inverse calculation, JFK to LHR
      double
        lat1 = 40.6, lon1 = -73.8, // JFK Airport
        lat2 = 51.6, lon2 = -0.5;  // LHR Airport
      double s12, azi12;
      rhumb.Inverse(lat1, lon1, lat2, lon2, s12, azi12);
      cout << s12 << " " << azi12 << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
