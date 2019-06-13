// Example of using the GeographicLib::SphericalHarmonic1 class

#include <iostream>
#include <exception>
#include <vector>
#include <GeographicLib/SphericalHarmonic1.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    int N = 3, N1 = 2;                  // The maxium degrees
    double ca[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
    vector<double> C(ca, ca + (N + 1) * (N + 2) / 2);
    double sa[] = {6, 5, 4, 3, 2, 1}; // sine coefficients
    vector<double> S(sa, sa + N * (N + 1) / 2);
    double cb[] = {1, 2, 3, 4, 5, 6};
    vector<double> C1(cb, cb + (N1 + 1) * (N1 + 2) / 2);
    double sb[] = {3, 2, 1};
    vector<double> S1(sb, sb + N1 * (N1 + 1) / 2);
    double a = 1;
    SphericalHarmonic1 h(C, S, N, C1, S1, N1, a);
    double tau = 0.1, x = 2, y = 3, z = 1;
    double v, vx, vy, vz;
    v = h(tau, x, y, z, vx, vy, vz);
    cout << v << " " << vx << " " << vy << " " << vz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
