// Example of using the GeographicLib::AuxAngle class.

#include <iostream>
#include <iomanip>
#include <exception>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/AuxAngle.hpp>

int main(int argc, const char* const argv[]) {
  try {
    typedef GeographicLib::AuxAngle angle;
    // Print table of parametric latitudes for f = 0.5
    double f = 0.5;
    std::cout << std::fixed << std::setprecision(4);
    for (int d = 0; d <= 90; d+=10) {
      angle phi(angle::degrees(d)), beta(phi);
      beta.y() *= (1 - f);
      std::cout << d << " " << beta.degrees() << "\n";
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
