// Example of using the GeographicLib::CircularEngine class

#include <iostream>
#include <exception>
#include <vector>
#include <GeographicLib/CircularEngine.hpp>
#include <GeographicLib/SphericalHarmonic.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  // This computes the same value as example-SphericalHarmonic.cpp using a
  // CircularEngine (which will be faster if many values on a circle of
  // latitude are to be found).
  try {
    int N = 3;                  // The maxium degree
    double ca[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
    vector<double> C(ca, ca + (N + 1) * (N + 2) / 2);
    double sa[] = {6, 5, 4, 3, 2, 1}; // sine coefficients
    vector<double> S(sa, sa + N * (N + 1) / 2);
    double a = 1;
    SphericalHarmonic h(C, S, N, a);
    double x = 2, y = 3, z = 1, p = Math::hypot(x, y);
    CircularEngine circ = h.Circle(p, z, true);
    double v, vx, vy, vz;
    v = circ(x/p, y/p, vx, vy, vz);
    cout << v << " " << vx << " " << vy << " " << vz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
