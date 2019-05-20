// Example of using the GeographicLib::SphericalHarmonic2 class

#include <iostream>
#include <exception>
#include <vector>
#include <GeographicLib/SphericalHarmonic2.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    int N = 3, N1 = 2, N2 = 1;                     // The maxium degrees
    double ca[] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
    vector<double> C(ca, ca + (N + 1) * (N + 2) / 2);
    double sa[] = {6, 5, 4, 3, 2, 1}; // sine coefficients
    vector<double> S(sa, sa + N * (N + 1) / 2);
    double cb[] = {1, 2, 3, 4, 5, 6};
    vector<double> C1(cb, cb + (N1 + 1) * (N1 + 2) / 2);
    double sb[] = {3, 2, 1};
    vector<double> S1(sb, sb + N1 * (N1 + 1) / 2);
    double cc[] = {2, 1};
    vector<double> C2(cc, cc + (N2 + 1));
    vector<double> S2;
    double a = 1;
    SphericalHarmonic2 h(C, S, N, N, N, C1, S1, N1, N1, N1,
                         C2, S2, N2, N2, 0, a);
    double tau1 = 0.1, tau2 = 0.05, x = 2, y = 3, z = 1;
    double v, vx, vy, vz;
    v = h(tau1, tau2, x, y, z, vx, vy, vz);
    cout << v << " " << vx << " " << vy << " " << vz << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
