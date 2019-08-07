// Example of using the GeographicLib::GravityModel class

#include <iostream>
#include <exception>
#include <GeographicLib/GravityModel.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    GravityModel grav("egm96");
    double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
    double gx, gy, gz;
    grav.Gravity(lat,lon, h, gx, gy, gz);
    cout << gx << " " << gy << " " << gz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
