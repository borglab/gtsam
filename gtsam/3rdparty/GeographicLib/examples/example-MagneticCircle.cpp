// Example of using the GeographicLib::MagneticCircle class

#include <iostream>
#include <exception>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/MagneticCircle.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    MagneticModel mag("wmm2010");
    double lat = 27.99, lon0 = 86.93, h = 8820, t = 2012; // Mt Everest
    {
      // Slow method of evaluating the values at several points on a circle of
      // latitude.
      for (int i = -5; i <= 5; ++i) {
        double lon = lon0 + i * 0.2;
        double Bx, By, Bz;
        mag(t, lat, lon, h, Bx, By, Bz);
        cout << lon << " " << Bx << " " << By << " " << Bz << "\n";
      }
    }
    {
      // Fast method of evaluating the values at several points on a circle of
      // latitude using MagneticCircle.
      MagneticCircle circ = mag.Circle(t, lat, h);
      for (int i = -5; i <= 5; ++i) {
        double lon = lon0 + i * 0.2;
        double Bx, By, Bz;
        circ(lon, Bx, By, Bz);
        cout << lon << " " << Bx << " " << By << " " << Bz << "\n";
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
