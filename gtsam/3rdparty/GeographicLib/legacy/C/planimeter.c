/**
 * @file planimeter.c
 * @brief A test program for geod_polygon_compute()
 **********************************************************************/

#include <stdio.h>
#include "geodesic.h"

#if defined(_MSC_VER)
/* Squelch warnings about scanf */
#  pragma warning (disable: 4996)
#endif

/**
 * A simple program to compute the area of a geodesic polygon.
 *
 * This program reads in lines with lat, lon for each vertex
 * of a polygon.  At the end of input, the program prints the number of
 * vertices, the perimeter of the polygon and its area (for the WGS84
 * ellipsoid).
 **********************************************************************/

int main() {
  double a = 6378137, f = 1/298.257223563; /* WGS84 */
  double lat, lon, A, P;
  int n;
  struct geod_geodesic g;
  struct geod_polygon p;
  geod_init(&g, a, f);
  geod_polygon_init(&p, 0);

  while (scanf("%lf %lf", &lat, &lon) == 2)
    geod_polygon_addpoint(&g, &p, lat, lon);
  n = geod_polygon_compute(&g, &p, 0, 1, &A, &P);
  printf("%d %.8f %.3f\n", n, P, A);
  return 0;
}
