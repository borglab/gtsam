// Example of using the GeographicLib::NormalGravity class

#include <iostream>
#include <exception>
#include <GeographicLib/NormalGravity.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    NormalGravity grav(Constants::WGS84_a(), Constants::WGS84_GM(),
                       Constants::WGS84_omega(), Constants::WGS84_f());
    // Alternatively: const NormalGravity& grav = NormalGravity::WGS84();
    double lat = 27.99, h = 8820; // Mt Everest
    double gammay, gammaz;
    grav.Gravity(lat, h, gammay, gammaz);
    cout << gammay << " " << gammaz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
