// Example of using the GeographicLib::SphericalEngine class

#include <iostream>
#include <exception>
#include <vector>
#include <GeographicLib/SphericalEngine.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  // See also example-SphericHarmonic.cpp
  try {
    int N = 3;                  // The maxium degree
    double ca[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
    vector<double> C(ca, ca + (N + 1) * (N + 2) / 2);
    double sa[] = {6, 5, 4, 3, 2, 1}; // sine coefficients
    vector<double> S(sa, sa + N * (N + 1) / 2);
    SphericalEngine::coeff c[1];
    c[0] = SphericalEngine::coeff(C, S, N);
    double f[] = {1};
    double x = 2, y = 3, z = 1, a = 1;
    double v, vx, vy, vz;
    v = SphericalEngine::Value<true, SphericalEngine::FULL, 1>
      (c, f, x, y, z, a, vx, vy, vz);
    cout << v << " " << vx << " " << vy << " " << vz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
