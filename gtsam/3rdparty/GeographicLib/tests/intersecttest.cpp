/**
 * \file intersecttest.cpp
 * \brief Test Intersect class
 *
 * Copyright (c) Charles Karney (2023) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <iostream>
#include <limits>
#include <string>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Intersect.hpp>

using namespace std;
using namespace GeographicLib;

typedef Math::real T;

static int checkEquals(T x, T y, T d) {
  if (fabs(x - y) <= d)
    return 0;
  cout << "checkEquals fails: " << x << " != " << y << " +/- " << d << "\n";
  return 1;
}

#define equatorialseg(lonx1, lonx2, lony1, lony2, px, py) do { \
    auto p1 = inter.Segment(0, lonx1, 0, lonx2,                \
                            0, lony1, 0, lony2, segmode),      \
      p2 = inter.Segment(0, lony1, 0, lony2,                   \
                         0, lonx1, 0, lonx2, segmode);         \
    int i = checkEquals(p1.first, px, eps) +                   \
      checkEquals(p1.second, py, eps) +                        \
      checkEquals(p2.first, py, eps) +                         \
      checkEquals(p2.second, px, eps);                         \
    if (i) cout << "ERROR at line " << __LINE__ << "\n";       \
    n += i;                                                    \
  } while (false)

int checkcoincident1() {
  int n = 0, segmode = 0;
  T a = T(180) / Math::pi(), eps = 1/T(1000000);
  for (int exact = 0; exact < 2; ++exact) {
    for (int fi = -1; fi <= 1; ++fi) {
      Geodesic geod(a, fi/T(10), exact != 0);
      Intersect inter(geod);
      equatorialseg(0, 40, -20, -10, -5, 15);
      equatorialseg(0, 40, -10, 0, 0, 10);
      equatorialseg(0, 40, -8, 2, 1, 9);
      equatorialseg(0, 40, -2, 8, 4, 6);
      equatorialseg(0, 40, 0, 10, 5, 5);
      equatorialseg(0, 40, 2, 12, 7, 5);
      equatorialseg(0, 40, 15, 25, 20, 5);
      equatorialseg(0, 40, 30, 40, 35, 5);
      equatorialseg(0, 40, 32, 42, 36, 4);
      equatorialseg(0, 40, 38, 48, 39, 1);
      equatorialseg(0, 40, 40, 50, 40, 0);
      equatorialseg(0, 40, 50, 60, 45, -5);
      equatorialseg(40, 0, -20, -10, 40+5, 15);
      equatorialseg(40, 0, -10, -0, 40, 10);
      equatorialseg(40, 0, -8, 2, 40-1, 9);
      equatorialseg(40, 0, -2, 8, 40-4, 6);
      equatorialseg(40, 0, 0, 10, 40-5, 5);
      equatorialseg(40, 0, 2, 12, 40-7, 5);
      equatorialseg(40, 0, 15, 25, 40-20, 5);
      equatorialseg(40, 0, 30, 40, 40-35, 5);
      equatorialseg(40, 0, 32, 42, 40-36, 4);
      equatorialseg(40, 0, 38, 48, 40-39, 1);
      equatorialseg(40, 0, 40, 50, 40-40, 0);
      equatorialseg(40, 0, 50, 60, 40-45, -5);
    }
  }
  return n;
}

int main() {
  int n = 0;
  n += checkcoincident1();
  if (n) {
    cout << n << " failure" << (n > 1 ? "s" : "") << "\n";
    return 1;
  }
}
