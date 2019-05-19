/**
 * @file inverse.c
 * @brief A test program for geod_inverse()
 **********************************************************************/

#include <stdio.h>
#include "geodesic.h"

#if defined(_MSC_VER)
/* Squelch warnings about scanf */
#  pragma warning (disable: 4996)
#endif

/**
 * A simple program to solve the inverse geodesic problem.
 *
 * This program reads in lines with lat1, lon1, lat2, lon2 and prints out lines
 * with azi1, azi2, s12 (for the WGS84 ellipsoid).
 **********************************************************************/

int main() {
  double a = 6378137, f = 1/298.257223563; /* WGS84 */
  double lat1, lon1, azi1, lat2, lon2, azi2, s12;
  struct geod_geodesic g;

  geod_init(&g, a, f);
  while (scanf("%lf %lf %lf %lf", &lat1, &lon1, &lat2, &lon2) == 4) {
    geod_inverse(&g, lat1, lon1, lat2, lon2, &s12, &azi1, &azi2);
    printf("%.15f %.15f %.10f\n", azi1, azi2, s12);
  }
  return 0;
}
