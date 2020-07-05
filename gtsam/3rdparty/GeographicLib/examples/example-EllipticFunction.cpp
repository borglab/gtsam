// Example of using the GeographicLib::EllipticFunction class

#include <iostream>
#include <iomanip>
#include <exception>
#include <cmath>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/EllipticFunction.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    EllipticFunction ell(0.1);  // parameter m = 0.1
    // See Abramowitz and Stegun, table 17.1
    cout << ell.K() << " " << ell.E() << "\n";
    double phi = 20, sn, cn;
    Math::sincosd(phi, sn ,cn);
    // See Abramowitz and Stegun, table 17.6 with
    // alpha = asin(sqrt(m)) = 18.43 deg and phi = 20 deg
    cout << ell.E(phi * Math::degree()) << " "
         << ell.E(sn, cn, ell.Delta(sn, cn))
         << "\n";
    // See Carlson 1995, Sec 3.
    cout << fixed << setprecision(16)
         << "RF(1,2,0)      = " << EllipticFunction::RF(1,2)      << "\n"
         << "RF(2,3,4)      = " << EllipticFunction::RF(2,3,4)    << "\n"
         << "RC(0,1/4)      = " << EllipticFunction::RC(0,0.25)   << "\n"
         << "RC(9/4,2)      = " << EllipticFunction::RC(2.25,2)   << "\n"
         << "RC(1/4,-2)     = " << EllipticFunction::RC(0.25,-2)  << "\n"
         << "RJ(0,1,2,3)    = " << EllipticFunction::RJ(0,1,2,3)  << "\n"
         << "RJ(2,3,4,5)    = " << EllipticFunction::RJ(2,3,4,5)  << "\n"
         << "RD(0,2,1)      = " << EllipticFunction::RD(0,2,1)    << "\n"
         << "RD(2,3,4)      = " << EllipticFunction::RD(2,3,4)    << "\n"
         << "RG(0,16,16)    = " << EllipticFunction::RG(16,16)    << "\n"
         << "RG(2,3,4)      = " << EllipticFunction::RG(2,3,4)    << "\n"
         << "RG(0,0.0796,4) = " << EllipticFunction::RG(0.0796,4) << "\n";
  }
  catch (const exception& e) {
    cout << "Caught exception: " << e.what() << "\n";
  }
}
