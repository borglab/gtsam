// Example of using the GeographicLib::GravityCircle class

#include <iostream>
#include <exception>
#include <GeographicLib/GravityModel.hpp>
#include <GeographicLib/GravityCircle.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    GravityModel grav("egm96");
    double lat = 27.99, lon0 = 86.93, h = 8820; // Mt Everest
    {
      // Slow method of evaluating the values at several points on a circle of
      // latitude.
      for (int i = -5; i <= 5; ++i) {
        double lon = lon0 + i * 0.2;
        double gx, gy, gz;
        grav.Gravity(lat, lon, h, gx, gy, gz);
        cout << lon << " " << gx << " " << gy << " " << gz << "\n";
      }
    }
    {
      // Fast method of evaluating the values at several points on a circle of
      // latitude using GravityCircle.
      GravityCircle circ = grav.Circle(lat, h);
      for (int i = -5; i <= 5; ++i) {
        double lon = lon0 + i * 0.2;
        double gx, gy, gz;
        circ.Gravity(lon, gx, gy, gz);
        cout << lon << " " << gx << " " << gy << " " << gz << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
