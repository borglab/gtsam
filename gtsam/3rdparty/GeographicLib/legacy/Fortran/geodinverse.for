*> @file geodinverse.for
*! @brief A test program for invers()

*> A simple program to solve the inverse geodesic problem.
*!
*! This program reads in lines with lat1, lon1, lon2, lat2 and prints
*! out lines with azi1, azi2, s12 (for the WGS84 ellipsoid).

      program geodinverse
      implicit none

      include 'geodesic.inc'

      double precision a, f, lat1, lon1, azi1, lat2, lon2, azi2, s12,
     +    dummy
      integer omask

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0

      omask = 0

 10   continue
      read(*, *, end=90, err=90) lat1, lon1, lat2, lon2
      call invers(a, f, lat1, lon1, lat2, lon2,
     +    s12, azi1, azi2, omask, dummy, dummy, dummy, dummy, dummy)
      print 20, azi1, azi2, s12
 20   format(f20.15, 1x, f20.15, 1x, f19.10)
      go to 10
 90   continue
      stop
      end
