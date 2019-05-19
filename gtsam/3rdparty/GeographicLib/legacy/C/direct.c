/**
 * @file direct.c
 * @brief A test program for geod_direct()
 **********************************************************************/

#include <stdio.h>
#include "geodesic.h"

#if defined(_MSC_VER)
/* Squelch warnings about scanf */
#  pragma warning (disable: 4996)
#endif

/**
 * A simple program to solve the direct geodesic problem.
 *
 * This program reads in lines with lat1, lon1, azi1, s12 and prints out lines
 * with lat2, lon2, azi2 (for the WGS84 ellipsoid).
 **********************************************************************/

int main() {
  double a = 6378137, f = 1/298.257223563; /* WGS84 */
  double lat1, lon1, azi1, lat2, lon2, azi2, s12;
  struct geod_geodesic g;

  geod_init(&g, a, f);
  while (scanf("%lf %lf %lf %lf", &lat1, &lon1, &azi1, &s12) == 4) {
    geod_direct(&g, lat1, lon1, azi1, s12, &lat2, &lon2, &azi2);
    printf("%.15f %.15f %.15f\n", lat2, lon2, azi2);
  }
  return 0;
}
