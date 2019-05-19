#include <stdio.h>
#include "cgeoid.h"

#if defined(_MSC_VER)
/* Squelch warnings about scanf */
#  pragma warning (disable: 4996)
#endif

int main() {
  double lat, lon, h;
  while(scanf("%lf %lf %lf", &lat, &lon, &h) == 3)
    printf("%.3f\n", HeightAboveEllipsoid(lat, lon, h));
}
