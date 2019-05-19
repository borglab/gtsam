*> @file geoddirect.for
*! @brief A test program for direct()

*> A simple program to solve the direct geodesic problem.
*!
*! This program reads in lines with lat1, lon1, azi1, s12 and prints out
*! lines with lat2, lon2, azi2 (for the WGS84 ellipsoid).

      program geoddirect
      implicit none

      include 'geodesic.inc'

      double precision a, f, lat1, lon1, azi1, lat2, lon2, azi2, s12,
     +    dummy1, dummy2, dummy3, dummy4, dummy5
      integer flags, omask

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0

      flags = 0
      omask = 0

 10   continue
      read(*, *, end=90, err=90) lat1, lon1, azi1, s12
      call direct(a, f, lat1, lon1, azi1, s12, flags,
     +    lat2, lon2, azi2, omask,
     +    dummy1, dummy2, dummy3, dummy4, dummy5)
      print 20, lat2, lon2, azi2
 20   format(1x, f20.15, 1x, f20.15, 1x, f20.15)
      go to 10
 90   continue

      stop
      end
