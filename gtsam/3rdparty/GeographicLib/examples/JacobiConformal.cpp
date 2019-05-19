// Example of using the GeographicLib::JacobiConformal class.

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/Utility.hpp>
#include "JacobiConformal.hpp"

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Utility::set_digits();
    // These parameters were derived from the EGM2008 geoid; see 2011-07-04
    // E-mail to PROJ.4 list, "Analyzing the bumps in the EGM2008 geoid".  The
    // longitude of the major axis is -15.  These are close to the values given
    // by Milan Bursa, Vladimira Fialova, "Parameters of the Earth's tri-axial
    // level ellipsoid", Studia Geophysica et Geodaetica 37(1), 1-13 (1993):
    //
    //    longitude of major axis = -14.93 +/- 0.05
    //    a = 6378171.36 +/- 0.30
    //    a/(a-c) = 297.7738 +/- 0.0003
    //    a/(a-b) = 91449 +/- 60
    // which gives: a = 6378171.36, b = 6378101.61, c = 6356751.84
    Math::real a = 6378137+35, b = 6378137-35, c = 6356752;
    JacobiConformal jc(a, b, c, a-b, b-c);
    cout  << fixed << setprecision(1)
          << "Ellipsoid parameters: a = "
          << a << ", b = " << b << ", c = " << c << "\n"
          << setprecision(10)
          << "Quadrants: x = " << jc.x() << ", y = " << jc.y() << "\n";
    cout << "Coordinates (angle x y) in degrees:\n";
    for (int i = 0; i <= 90; i += 5) {
      Math::real omg = i, bet = i;
      cout << i << " " << jc.x(omg) << " " << jc.y(bet) << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
