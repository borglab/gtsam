*> @file geodesic.inc
*! @brief The interface file for the geodesic routines in Fortran
*!
*! Optinally insert \code
*! include 'geodesic.inc' \endcode
*! into the declaration portion of a subroutine that uses this library.
*!
*! See geodesic.for for documentation on these routines.

      interface

        subroutine direct(a, f, lat1, lon1, azi1, s12a12, arcmod,
     +      lat2, lon2, azi2, omask, a12s12, m12, MM12, MM21, SS12)
        double precision, intent(in) :: a, f, lat1, lon1, azi1, s12a12
        logical, intent(in) :: arcmod
        integer, intent(in) :: omask
        double precision, intent(out) :: lat2, lon2, azi2
        double precision, intent(out) :: a12s12, m12, MM12, MM21, SS12
        end subroutine direct

        subroutine invers(a, f, lat1, lon1, lat2, lon2,
     +      s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
        double precision, intent(in) :: a, f, lat1, lon1, lat2, lon2
        integer, intent(in) :: omask
        double precision, intent(out) :: s12, azi1, azi2
        double precision, intent(out) :: a12, m12, MM12, MM21, SS12
        end subroutine invers

        subroutine area(a, f, lats, lons, n, AA, PP)
        integer, intent(in) :: n
        double precision, intent(in) :: a, f, lats(n), lons(n)
        double precision, intent(out) :: AA, PP
        end subroutine area

      end interface
