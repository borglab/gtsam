// Write the coefficient files needed for approximating the normal gravity
// field with a GravityModel.  WARNING: this creates files, wgs84.egm.cof and
// grs80.egm.cof, in the current directory.

#include <cmath>
#include <fstream>
#include <iostream>
#include <GeographicLib/NormalGravity.hpp>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    Utility::set_digits();
    const char* filenames[] = {"wgs84.egm.cof", "grs80.egm.cof"};
    const char* ids[] = {"WGS1984A", "GRS1980A"};
    for (int grs80 = 0; grs80 < 2; ++grs80) {
      ofstream file(filenames[grs80], ios::binary);
      Utility::writearray<char, char, false>(file, ids[grs80], 8);
      const int N = 20, M = 0,
        cnum = (M + 1) * (2 * N - M + 2) / 2; // cnum = N + 1
      vector<int> num(2);
      num[0] = N; num[1] = M;
      Utility::writearray<int, int, false>(file, num);
      vector<Math::real> c(cnum, 0);
      const NormalGravity& earth(grs80 ? NormalGravity::GRS80() :
                                 NormalGravity::WGS84());
      for (int n = 2; n <= N; n += 2)
        c[n] = - earth.DynamicalFormFactor(n) / sqrt(Math::real(2*n + 1));
      Utility::writearray<double, Math::real, false>(file, c);
      num[0] = num[1] = -1;
      Utility::writearray<int, int, false>(file, num);
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  catch (...) {
    cerr << "Caught unknown exception\n";
    return 1;
  }
}
