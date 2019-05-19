*> @file geodtest.for
*! @brief Test suite for the geodesic routines in Fortran
*!
*! Run these tests by configuring with cmake and running "make test".
*!
*! Copyright (c) Charles Karney (2015-2017) <charles@karney.com> and
*! licensed under the MIT/X11 License.  For more information, see
*! https://geographiclib.sourceforge.io/

*> @cond SKIP

      block data tests

      integer j
      double precision tstdat(20, 12)
      common /tstcom/ tstdat
      data (tstdat(1,j), j = 1,12) /
     +    35.60777d0,-139.44815d0,111.098748429560326d0,
     +    -11.17491d0,-69.95921d0,129.289270889708762d0,
     +    8935244.5604818305d0,80.50729714281974d0,6273170.2055303837d0,
     +    0.16606318447386067d0,0.16479116945612937d0,
     +    12841384694976.432d0 /
      data (tstdat(2,j), j = 1,12) /
     +    55.52454d0,106.05087d0,22.020059880982801d0,
     +    77.03196d0,197.18234d0,109.112041110671519d0,
     +    4105086.1713924406d0,36.892740690445894d0,
     +    3828869.3344387607d0,
     +    0.80076349608092607d0,0.80101006984201008d0,
     +    61674961290615.615d0 /
      data (tstdat(3,j), j = 1,12) /
     +    -21.97856d0,142.59065d0,-32.44456876433189d0,
     +    41.84138d0,98.56635d0,-41.84359951440466d0,
     +    8394328.894657671d0,75.62930491011522d0,6161154.5773110616d0,
     +    0.24816339233950381d0,0.24930251203627892d0,
     +    -6637997720646.717d0 /
      data (tstdat(4,j), j = 1,12) /
     +    -66.99028d0,112.2363d0,173.73491240878403d0,
     +    -12.70631d0,285.90344d0,2.512956620913668d0,
     +    11150344.2312080241d0,100.278634181155759d0,
     +    6289939.5670446687d0,
     +    -0.17199490274700385d0,-0.17722569526345708d0,
     +    -121287239862139.744d0 /
      data (tstdat(5,j), j = 1,12) /
     +    -17.42761d0,173.34268d0,-159.033557661192928d0,
     +    -15.84784d0,5.93557d0,-20.787484651536988d0,
     +    16076603.1631180673d0,144.640108810286253d0,
     +    3732902.1583877189d0,
     +    -0.81273638700070476d0,-0.81299800519154474d0,
     +    97825992354058.708d0 /
      data (tstdat(6,j), j = 1,12) /
     +    32.84994d0,48.28919d0,150.492927788121982d0,
     +    -56.28556d0,202.29132d0,48.113449399816759d0,
     +    16727068.9438164461d0,150.565799985466607d0,
     +    3147838.1910180939d0,
     +    -0.87334918086923126d0,-0.86505036767110637d0,
     +    -72445258525585.010d0 /
      data (tstdat(7,j), j = 1,12) /
     +    6.96833d0,52.74123d0,92.581585386317712d0,
     +    -7.39675d0,206.17291d0,90.721692165923907d0,
     +    17102477.2496958388d0,154.147366239113561d0,
     +    2772035.6169917581d0,
     +    -0.89991282520302447d0,-0.89986892177110739d0,
     +    -1311796973197.995d0 /
      data (tstdat(8,j), j = 1,12) /
     +    -50.56724d0,-16.30485d0,-105.439679907590164d0,
     +    -33.56571d0,-94.97412d0,-47.348547835650331d0,
     +    6455670.5118668696d0,58.083719495371259d0,
     +    5409150.7979815838d0,
     +    0.53053508035997263d0,0.52988722644436602d0,
     +    41071447902810.047d0 /
      data (tstdat(9,j), j = 1,12) /
     +    -58.93002d0,-8.90775d0,140.965397902500679d0,
     +    -8.91104d0,133.13503d0,19.255429433416599d0,
     +    11756066.0219864627d0,105.755691241406877d0,
     +    6151101.2270708536d0,
     +    -0.26548622269867183d0,-0.27068483874510741d0,
     +    -86143460552774.735d0 /
      data (tstdat(10,j), j = 1,12) /
     +    -68.82867d0,-74.28391d0,93.774347763114881d0,
     +    -50.63005d0,-8.36685d0,34.65564085411343d0,
     +    3956936.926063544d0,35.572254987389284d0,3708890.9544062657d0,
     +    0.81443963736383502d0,0.81420859815358342d0,
     +    -41845309450093.787d0 /
      data (tstdat(11,j), j = 1,12) /
     +    -10.62672d0,-32.0898d0,-86.426713286747751d0,
     +    5.883d0,-134.31681d0,-80.473780971034875d0,
     +    11470869.3864563009d0,103.387395634504061d0,
     +    6184411.6622659713d0,
     +    -0.23138683500430237d0,-0.23155097622286792d0,
     +    4198803992123.548d0 /
      data (tstdat(12,j), j = 1,12) /
     +    -21.76221d0,166.90563d0,29.319421206936428d0,
     +    48.72884d0,213.97627d0,43.508671946410168d0,
     +    9098627.3986554915d0,81.963476716121964d0,
     +    6299240.9166992283d0,
     +    0.13965943368590333d0,0.14152969707656796d0,
     +    10024709850277.476d0 /
      data (tstdat(13,j), j = 1,12) /
     +    -19.79938d0,-174.47484d0,71.167275780171533d0,
     +    -11.99349d0,-154.35109d0,65.589099775199228d0,
     +    2319004.8601169389d0,20.896611684802389d0,
     +    2267960.8703918325d0,
     +    0.93427001867125849d0,0.93424887135032789d0,
     +    -3935477535005.785d0 /
      data (tstdat(14,j), j = 1,12) /
     +    -11.95887d0,-116.94513d0,92.712619830452549d0,
     +    4.57352d0,7.16501d0,78.64960934409585d0,
     +    13834722.5801401374d0,124.688684161089762d0,
     +    5228093.177931598d0,
     +    -0.56879356755666463d0,-0.56918731952397221d0,
     +    -9919582785894.853d0 /
      data (tstdat(15,j), j = 1,12) /
     +    -87.85331d0,85.66836d0,-65.120313040242748d0,
     +    66.48646d0,16.09921d0,-4.888658719272296d0,
     +    17286615.3147144645d0,155.58592449699137d0,
     +    2635887.4729110181d0,
     +    -0.90697975771398578d0,-0.91095608883042767d0,
     +    42667211366919.534d0 /
      data (tstdat(16,j), j = 1,12) /
     +    1.74708d0,128.32011d0,-101.584843631173858d0,
     +    -11.16617d0,11.87109d0,-86.325793296437476d0,
     +    12942901.1241347408d0,116.650512484301857d0,
     +    5682744.8413270572d0,
     +    -0.44857868222697644d0,-0.44824490340007729d0,
     +    10763055294345.653d0 /
      data (tstdat(17,j), j = 1,12) /
     +    -25.72959d0,-144.90758d0,-153.647468693117198d0,
     +    -57.70581d0,-269.17879d0,-48.343983158876487d0,
     +    9413446.7452453107d0,84.664533838404295d0,
     +    6356176.6898881281d0,
     +    0.09492245755254703d0,0.09737058264766572d0,
     +    74515122850712.444d0 /
      data (tstdat(18,j), j = 1,12) /
     +    -41.22777d0,122.32875d0,14.285113402275739d0,
     +    -7.57291d0,130.37946d0,10.805303085187369d0,
     +    3812686.035106021d0,34.34330804743883d0,3588703.8812128856d0,
     +    0.82605222593217889d0,0.82572158200920196d0,
     +    -2456961531057.857d0 /
      data (tstdat(19,j), j = 1,12) /
     +    11.01307d0,138.25278d0,79.43682622782374d0,
     +    6.62726d0,247.05981d0,103.708090215522657d0,
     +    11911190.819018408d0,107.341669954114577d0,
     +    6070904.722786735d0,
     +    -0.29767608923657404d0,-0.29785143390252321d0,
     +    17121631423099.696d0 /
      data (tstdat(20,j), j = 1,12) /
     +    -29.47124d0,95.14681d0,-163.779130441688382d0,
     +    -27.46601d0,-69.15955d0,-15.909335945554969d0,
     +    13487015.8381145492d0,121.294026715742277d0,
     +    5481428.9945736388d0,
     +    -0.51527225545373252d0,-0.51556587964721788d0,
     +    104679964020340.318d0 /
      end

      integer function assert(x, y, d)
      double precision x, y, d

      if (abs(x - y) .le. d) then
        assert = 0
      else
        assert = 1
        print 10, x, y, d
 10     format(1x, 'assert fails: ',
     +      g14.7, ' != ', g14.7, ' +/- ', g10.3)
      end if

      return
      end

      integer function tstinv()
      double precision tstdat(20, 12)
      common /tstcom/ tstdat
      double precision lat1, lon1, azi1, lat2, lon2, azi2,
     +    s12, a12, m12, MM12, MM21, SS12
      double precision azi1a, azi2a, s12a, a12a,
     +    m12a, MM12a, MM21a, SS12a
      double precision a, f
      integer r, assert, i, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 1 + 2 + 4 + 8
      r = 0

      do i = 1,20
        lat1 = tstdat(i, 1)
        lon1 = tstdat(i, 2)
        azi1 = tstdat(i, 3)
        lat2 = tstdat(i, 4)
        lon2 = tstdat(i, 5)
        azi2 = tstdat(i, 6)
        s12 = tstdat(i, 7)
        a12 = tstdat(i, 8)
        m12 = tstdat(i, 9)
        MM12 = tstdat(i, 10)
        MM21 = tstdat(i, 11)
        SS12 = tstdat(i, 12)
        call invers(a, f, lat1, lon1, lat2, lon2,
     +      s12a, azi1a, azi2a, omask, a12a, m12a, MM12a, MM21a, SS12a)
        r = r + assert(azi1, azi1a, 1d-13)
        r = r + assert(azi2, azi2a, 1d-13)
        r = r + assert(s12, s12a, 1d-8)
        r = r + assert(a12, a12a, 1d-13)
        r = r + assert(m12, m12a, 1d-8)
        r = r + assert(MM12, MM12a, 1d-15)
        r = r + assert(MM21, MM21a, 1d-15)
        r = r + assert(SS12, SS12a, 0.1d0)
      end do

      tstinv = r
      return
      end

      integer function tstdir()
      double precision tstdat(20, 12)
      common /tstcom/ tstdat
      double precision lat1, lon1, azi1, lat2, lon2, azi2,
     +    s12, a12, m12, MM12, MM21, SS12
      double precision lat2a, lon2a, azi2a, a12a,
     +    m12a, MM12a, MM21a, SS12a
      double precision a, f
      integer r, assert, i, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 1 + 2 + 4 + 8
      flags = 2
      r = 0

      do i = 1,20
        lat1 = tstdat(i, 1)
        lon1 = tstdat(i, 2)
        azi1 = tstdat(i, 3)
        lat2 = tstdat(i, 4)
        lon2 = tstdat(i, 5)
        azi2 = tstdat(i, 6)
        s12 = tstdat(i, 7)
        a12 = tstdat(i, 8)
        m12 = tstdat(i, 9)
        MM12 = tstdat(i, 10)
        MM21 = tstdat(i, 11)
        SS12 = tstdat(i, 12)
        call direct(a, f, lat1, lon1, azi1, s12, flags,
     +    lat2a, lon2a, azi2a, omask, a12a, m12a, MM12a, MM21a, SS12a)
        r = r + assert(lat2, lat2a, 1d-13)
        r = r + assert(lon2, lon2a, 1d-13)
        r = r + assert(azi2, azi2a, 1d-13)
        r = r + assert(a12, a12a, 1d-13)
        r = r + assert(m12, m12a, 1d-8)
        r = r + assert(MM12, MM12a, 1d-15)
        r = r + assert(MM21, MM21a, 1d-15)
        r = r + assert(SS12, SS12a, 0.1d0)
      end do

      tstdir = r
      return
      end

      integer function tstarc()
      double precision tstdat(20, 12)
      common /tstcom/ tstdat
      double precision lat1, lon1, azi1, lat2, lon2, azi2,
     +    s12, a12, m12, MM12, MM21, SS12
      double precision lat2a, lon2a, azi2a, s12a,
     +    m12a, MM12a, MM21a, SS12a
      double precision a, f
      integer r, assert, i, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 1 + 2 + 4 + 8
      flags = 1 + 2
      r = 0

      do i = 1,20
        lat1 = tstdat(i, 1)
        lon1 = tstdat(i, 2)
        azi1 = tstdat(i, 3)
        lat2 = tstdat(i, 4)
        lon2 = tstdat(i, 5)
        azi2 = tstdat(i, 6)
        s12 = tstdat(i, 7)
        a12 = tstdat(i, 8)
        m12 = tstdat(i, 9)
        MM12 = tstdat(i, 10)
        MM21 = tstdat(i, 11)
        SS12 = tstdat(i, 12)
        call direct(a, f, lat1, lon1, azi1, a12, flags,
     +    lat2a, lon2a, azi2a, omask, s12a, m12a, MM12a, MM21a, SS12a)
        r = r + assert(lat2, lat2a, 1d-13)
        r = r + assert(lon2, lon2a, 1d-13)
        r = r + assert(azi2, azi2a, 1d-13)
        r = r + assert(s12, s12a, 1d-8)
        r = r + assert(m12, m12a, 1d-8)
        r = r + assert(MM12, MM12a, 1d-15)
        r = r + assert(MM21, MM21a, 1d-15)
        r = r + assert(SS12, SS12a, 0.1d0)
      end do

      tstarc = r
      return
      end

      integer function notnan(x)
      double precision x
      if (x .eq. x) then
        notnan = 1
      else
        notnan = 0
      end if

      return
      end

      integer function tstg0()
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 40.6d0, -73.8d0, 49.01666667d0, 2.55d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 53.47022d0, 0.5d-5)
      r = r + assert(azi2, 111.59367d0, 0.5d-5)
      r = r + assert(s12, 5853226d0, 0.5d0)

      tstg0 = r
      return
      end

      integer function tstg1()
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      flags = 0
      r = 0
      call direct(a, f, 40.63972222d0, -73.77888889d0, 53.5d0, 5850d3,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(lat2, 49.01467d0, 0.5d-5)
      r = r + assert(lon2, 2.56106d0, 0.5d-5)
      r = r + assert(azi2, 111.62947d0, 0.5d-5)

      tstg1 = r
      return
      end

      integer function tstg2()
* Check fix for antipodal prolate bug found 2010-09-04
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

      a = 6.4d6
      f = -1/150d0
      omask = 0
      r = 0
      call invers(a, f, 0.07476d0, 0d0, -0.07476d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00078d0, 0.5d-5)
      r = r + assert(azi2, 90.00078d0, 0.5d-5)
      r = r + assert(s12, 20106193d0, 0.5d0)
      call invers(a, f, 0.1d0, 0d0, -0.1d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00105d0, 0.5d-5)
      r = r + assert(azi2, 90.00105d0, 0.5d-5)
      r = r + assert(s12, 20106193d0, 0.5d0)

      tstg2 = r
      return
      end

      integer function tstg4()
* Check fix for short line bug found 2010-05-21
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f,
     +    36.493349428792d0, 0d0, 36.49334942879201d0, .0000008d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 0.072d0, 0.5d-3)

      tstg4 = r
      return
      end

      integer function tstg5()
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      flags = 0
      r = 0
      call direct(a, f, 0.01777745589997d0, 30d0, 0d0, 10d6,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      if (lon2 .lt. 0) then
        r = r + assert(lon2, -150d0, 0.5d-5)
        r = r + assert(abs(azi2), 180d0, 0.5d-5)
      else
        r = r + assert(lon2, 30d0, 0.5d-5)
        r = r + assert(azi2, 0d0, 0.5d-5)
      end if

      tstg5 = r
      return
      end

      integer function tstg6()
* Check fix for volatile sbet12a bug found 2011-06-25 (gcc 4.4d0.4d0
* x86 -O3).  Found again on 2012-03-27 with tdm-mingw32 (g++ 4.6d0.1d0).
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 88.202499451857d0, 0d0,
     +    -88.202499451857d0, 179.981022032992859592d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 20003898.214d0, 0.5d-3)
      call invers(a, f, 89.262080389218d0, 0d0,
     +    -89.262080389218d0, 179.992207982775375662d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 20003925.854d0, 0.5d-3)
      call invers(a, f, 89.333123580033d0, 0d0,
     +    -89.333123580032997687d0, 179.99295812360148422d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 20003926.881d0, 0.5d-3)

      tstg6 = r
      return
      end

      integer function tstg9()
* Check fix for volatile x bug found 2011-06-25 (gcc 4.4d0.4d0 x86 -O3)
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 56.320923501171d0, 0d0,
     +    -56.320923501171d0, 179.664747671772880215d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 19993558.287d0, 0.5d-3)

      tstg9 = r
      return
      end

      integer function tstg10()
* Check fix for adjust tol1_ bug found 2011-06-25 (Visual Studio 10 rel
* + debug)
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 52.784459512564d0, 0d0,
     +    -52.784459512563990912d0, 179.634407464943777557d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 19991596.095d0, 0.5d-3)

      tstg10 = r
      return
      end

      integer function tstg11()
* Check fix for bet2 = -bet1 bug found 2011-06-25 (Visual Studio 10 rel
* + debug)
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 48.522876735459d0, 0d0,
     +    -48.52287673545898293d0, 179.599720456223079643d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(s12, 19989144.774d0, 0.5d-3)

      tstg11 = r
      return
      end

      integer function tstg12()
* Check fix for inverse geodesics on extreme prolate/oblate ellipsoids
* Reported 2012-08-29 Stefan Guenther <stefan.gunther@embl.de>; fixed
* 2012-10-07
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

      a = 89.8d0
      f = -1.83d0
      omask = 0
      r = 0
      call invers(a, f, 0d0, 0d0, -10d0, 160d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 120.27d0, 1d-2)
      r = r + assert(azi2, 105.15d0, 1d-2)
      r = r + assert(s12, 266.7d0, 1d-1)

      tstg12 = r
      return
      end

      integer function tstg14()
* Check fix for inverse ignoring lon12 = nan
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f, LatFix
      integer r, notnan, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 0d0, 0d0, 1d0, LatFix(91d0),
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + notnan(azi1)
      r = r + notnan(azi2)
      r = r + notnan(s12)
      tstg14 = r
      return
      end

      integer function tstg15()
* Initial implementation of Math::eatanhe was wrong for e^2 < 0.  This
* checks that this is fixed.
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

      a = 6.4d6
      f = -1/150.0d0
      omask = 8
      flags = 0
      r = 0
      call direct(a, f, 1d0, 2d0, 3d0, 4d0,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(SS12, 23700d0, 0.5d0)

      tstg15 = r
      return
      end

      integer function tstg17()
* Check fix for LONG_UNROLL bug found on 2015-05-07
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      flags = 2
      r = 0
      call direct(a, f, 40d0, -75d0, -10d0, 2d7,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(lat2, -39d0, 1d0)
      r = r + assert(lon2, -254d0, 1d0)
      r = r + assert(azi2, -170d0, 1d0)
      flags = 0
      call direct(a, f, 40d0, -75d0, -10d0, 2d7,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(lat2, -39d0, 1d0)
      r = r + assert(lon2, 105d0, 1d0)
      r = r + assert(azi2, -170d0, 1d0)

      tstg17 = r
      return
      end

      integer function tstg26()
* Check 0/0 problem with area calculation on sphere 2015-09-08
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

      a = 6.4d6
      f = 0
      omask = 8
      r = 0
      call invers(a, f, 1d0, 2d0, 3d0, 4d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(SS12, 49911046115.0d0, 0.5d0)

      tstg26 = r
      return
      end

      integer function tstg28()
* Check fix for LONG_UNROLL bug found on 2015-05-07
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

      a = 6.4d6
      f = 0.1d0
      omask = 1 + 2 + 4 + 8
      flags = 0
      r = 0
      call direct(a, f, 1d0, 2d0, 10d0, 5d6,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(a12, 48.55570690d0, 0.5d-8)

      tstg28 = r
      return
      end

      integer function tstg33()
* Check max(-0.0,+0.0) issues 2015-08-22 (triggered by bugs in Octave --
* sind(-0.0) = +0.0 -- and in some version of Visual Studio --
* fmod(-0.0, 360.0) = +0.0.
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 0d0, 0d0, 0d0, 179d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00000d0, 0.5d-5)
      r = r + assert(azi2, 90.00000d0, 0.5d-5)
      r = r + assert(s12, 19926189d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 0d0, 179.5d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 55.96650d0, 0.5d-5)
      r = r + assert(azi2, 124.03350d0, 0.5d-5)
      r = r + assert(s12, 19980862d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 0d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.00000d0, 0.5d-5)
      r = r + assert(abs(azi2), 180.00000d0, 0.5d-5)
      r = r + assert(s12, 20003931d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 1d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.00000d0, 0.5d-5)
      r = r + assert(abs(azi2), 180.00000d0, 0.5d-5)
      r = r + assert(s12, 19893357d0, 0.5d0)
      a = 6.4d6
      f = 0
      call invers(a, f, 0d0, 0d0, 0d0, 179d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00000d0, 0.5d-5)
      r = r + assert(azi2, 90.00000d0, 0.5d-5)
      r = r + assert(s12, 19994492d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 0d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.00000d0, 0.5d-5)
      r = r + assert(abs(azi2), 180.00000d0, 0.5d-5)
      r = r + assert(s12, 20106193d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 1d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.00000d0, 0.5d-5)
      r = r + assert(abs(azi2), 180.00000d0, 0.5d-5)
      r = r + assert(s12, 19994492d0, 0.5d0)
      a = 6.4d6
      f = -1/300.0d0
      call invers(a, f, 0d0, 0d0, 0d0, 179d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00000d0, 0.5d-5)
      r = r + assert(azi2, 90.00000d0, 0.5d-5)
      r = r + assert(s12, 19994492d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 0d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 90.00000d0, 0.5d-5)
      r = r + assert(azi2, 90.00000d0, 0.5d-5)
      r = r + assert(s12, 20106193d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 0.5d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 33.02493d0, 0.5d-5)
      r = r + assert(azi2, 146.97364d0, 0.5d-5)
      r = r + assert(s12, 20082617d0, 0.5d0)
      call invers(a, f, 0d0, 0d0, 1d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.00000d0, 0.5d-5)
      r = r + assert(abs(azi2), 180.00000d0, 0.5d-5)
      r = r + assert(s12, 20027270d0, 0.5d0)

      tstg33 = r
      return
      end

      integer function tstg55()
* Check fix for nan + point on equator or pole not returning all nans in
* Geodesic::Inverse, found 2015-09-23.
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, notnan, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 91d0, 0d0, 0d0, 90d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + notnan(azi1)
      r = r + notnan(azi2)
      r = r + notnan(s12)
      call invers(a, f, 91d0, 0d0, 90d0, 9d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + notnan(azi1)
      r = r + notnan(azi2)
      r = r + notnan(s12)
      tstg55 = r
      return
      end

      integer function tstg59()
* Check for points close with longitudes close to 180 deg apart.
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 5d0, 0.00000000000001d0, 10d0, 180d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 0.000000000000035d0, 1.5d-14);
      r = r + assert(azi2, 179.99999999999996d0, 1.5d-14);
      r = r + assert(s12, 18345191.174332713d0, 2.5d-9);
      tstg59 = r
      return
      end

      integer function tstg61()
* Make sure small negative azimuths are west-going
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      flags = 2
      r = 0
      call direct(a, f, 45d0, 0d0, -0.000000000000000003d0, 1d7,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(lat2, 45.30632d0, 0.5d-5)
      r = r + assert(lon2, -180d0, 0.5d-5)
      r = r + assert(abs(azi2), 180d0, 0.5d-5)

      tstg61 = r
      return
      end

      integer function tstg73()
* Check for backwards from the pole bug reported by Anon on 2016-02-13.
* This only affected the Java implementation.  It was introduced in Java
* version 1.44 and fixed in 1.46-SNAPSHOT on 2016-01-17.
      double precision lat2, lon2, azi2, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask, flags
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      flags = 0
      r = 0
      call direct(a, f, 90d0, 10d0, 180d0, -1d6,
     +    flags, lat2, lon2, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(lat2, 81.04623d0, 0.5d-5)
      r = r + assert(lon2, -170d0, 0.5d-5)
      r = r + assert(azi2, 0d0, 0.5d-5)

      tstg73 = r
      return
      end

      integer function tstg74()
* Check fix for inaccurate areas, bug introduced in v1.46, fixed
* 2015-10-16.
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 1 + 2 + 4 + 8
      r = 0
      call invers(a, f, 54.1589d0, 15.3872d0, 54.1591d0, 15.3877d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 55.723110355d0, 5d-9);
      r = r + assert(azi2, 55.723515675d0, 5d-9);
      r = r + assert(s12,  39.527686385d0, 5d-9);
      r = r + assert(a12,   0.000355495d0, 5d-9);
      r = r + assert(m12,  39.527686385d0, 5d-9);
      r = r + assert(MM12,  0.999999995d0, 5d-9);
      r = r + assert(MM21,  0.999999995d0, 5d-9);
      r = r + assert(SS12, 286698586.30197d0, 5d-4);
      tstg74 = r
      return
      end

      integer function tstg76()
* The distance from Wellington and Salamanca (a classic failure of
* Vincenty
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f,
     +    -(41+19/60d0), 174+49/60d0, 40+58/60d0, -(5+30/60d0),
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1, 160.39137649664d0, 0.5d-11)
      r = r + assert(azi2,  19.50042925176d0, 0.5d-11)
      r = r + assert(s12,  19960543.857179d0, 0.5d-6)
      tstg76 = r
      return
      end

      integer function tstg78()
* An example where the NGS calculator fails to converge
      double precision azi1, azi2, s12, a12, m12, MM12, MM21, SS12
      double precision a, f
      integer r, assert, omask
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      omask = 0
      r = 0
      call invers(a, f, 27.2d0, 0d0, -27.1d0, 179.5d0,
     +    s12, azi1, azi2, omask, a12, m12, MM12, MM21, SS12)
      r = r + assert(azi1,  45.82468716758d0, 0.5d-11)
      r = r + assert(azi2, 134.22776532670d0, 0.5d-11)
      r = r + assert(s12,  19974354.765767d0, 0.5d-6)
      tstg78 = r
      return
      end

      integer function tstp0()
* Check fix for pole-encircling bug found 2011-03-16
      double precision lata(4), lona(4)
      data lata / 89d0, 89d0, 89d0, 89d0 /
      data lona / 0d0, 90d0, 180d0, 270d0 /
      double precision latb(4), lonb(4)
      data latb / -89d0, -89d0, -89d0, -89d0 /
      data lonb / 0d0, 90d0, 180d0, 270d0 /
      double precision latc(4), lonc(4)
      data latc / 0d0, -1d0, 0d0, 1d0 /
      data lonc / -1d0, 0d0, 1d0, 0d0 /
      double precision latd(3), lond(3)
      data latd / 90d0, 0d0, 0d0 /
      data lond / 0d0, 0d0, 90d0 /
      double precision a, f, AA, PP
      integer r, assert
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      r = 0

      call area(a, f, lata, lona, 4, AA, PP)
      r = r + assert(PP, 631819.8745d0, 1d-4)
      r = r + assert(AA, 24952305678.0d0, 1d0)

      call area(a, f, latb, lonb, 4, AA, PP)
      r = r + assert(PP, 631819.8745d0, 1d-4)
      r = r + assert(AA, -24952305678.0d0, 1d0)

      call area(a, f, latc, lonc, 4, AA, PP)
      r = r + assert(PP, 627598.2731d0, 1d-4)
      r = r + assert(AA, 24619419146.0d0, 1d0)

      call area(a, f, latd, lond, 3, AA, PP)
      r = r + assert(PP, 30022685d0, 1d0)
      r = r + assert(AA, 63758202715511.0d0, 1d0)

      tstp0 = r
      return
      end

      integer function tstp5()
* Check fix for Planimeter pole crossing bug found 2011-06-24
      double precision lat(3), lon(3)
      data lat / 89d0, 89d0, 89d0 /
      data lon / 0.1d0, 90.1d0, -179.9d0 /
      double precision a, f, AA, PP
      integer r, assert
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      r = 0

      call area(a, f, lat, lon, 3, AA, PP)
      r = r + assert(PP, 539297d0, 1d0)
      r = r + assert(AA, 12476152838.5d0, 1d0)

      tstp5 = r
      return
      end

      integer function tstp6()
* Check fix for pole-encircling bug found 2011-03-16
      double precision lata(3), lona(3)
      data lata / 9d0, 9d0, 9d0 /
      data lona / -0.00000000000001d0, 180d0, 0d0 /
      double precision latb(3), lonb(3)
      data latb / 9d0, 9d0, 9d0 /
      data lonb / 0.00000000000001d0, 0d0, 180d0 /
      double precision latc(3), lonc(3)
      data latc / 9d0, 9d0, 9d0 /
      data lonc / 0.00000000000001d0, 180d0, 0d0 /
      double precision latd(3), lond(3)
      data latd / 9d0, 9d0, 9d0 /
      data lond / -0.00000000000001d0, 0d0, 180d0 /
      double precision a, f, AA, PP
      integer r, assert
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      r = 0

      call area(a, f, lata, lona, 3, AA, PP)
      r = r + assert(PP, 36026861d0, 1d0)
      r = r + assert(AA, 0d0, 1d0)

      tstp6 = r
      return
      end

      integer function tstp12()
* AA of arctic circle (not really -- adjunct to rhumb-AA test)
      double precision lat(2), lon(2)
      data lat / 66.562222222d0, 66.562222222d0 /
      data lon / 0d0, 180d0 /
      double precision a, f, AA, PP
      integer r, assert
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      r = 0

      call area(a, f, lat, lon, 2, AA, PP)
      r = r + assert(PP, 10465729d0, 1d0)
      r = r + assert(AA, 0d0, 1d0)

      tstp12 = r
      return
      end

      integer function tstp13()
* Check encircling pole twice
      double precision lat(6), lon(6)
      data lat / 89d0, 89d0, 89d0, 89d0, 89d0, 89d0 /
      data lon / -360d0, -240d0, -120d0, 0d0, 120d0, 240d0 /
      double precision a, f, AA, PP
      integer r, assert
      include 'geodesic.inc'

* WGS84 values
      a = 6378137d0
      f = 1/298.257223563d0
      r = 0

      call area(a, f, lat, lon, 6, AA, PP)
      r = r + assert(PP, 1160741d0, 1d0)
      r = r + assert(AA, 32415230256.0d0, 1d0)

      tstp13 = r
      return
      end

      program geodtest
      integer n, i
      integer tstinv, tstdir, tstarc,
     +    tstg0, tstg1, tstg2, tstg5, tstg6, tstg9, tstg10, tstg11,
     +    tstg12, tstg14, tstg15, tstg17, tstg26, tstg28, tstg33,
     +    tstg55, tstg59, tstg61, tstg73, tstg74, tstg76, tstg78,
     +    tstp0, tstp5, tstp6, tstp12, tstp13

      n = 0
      i = tstinv()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstinv fail:', i
      end if
      i = tstdir()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstdir fail:', i
      end if
      i = tstarc()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstarc fail:', i
      end if
      i = tstg0()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg0 fail:', i
      end if
      i = tstg1()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg1 fail:', i
      end if
      i = tstg2()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg2 fail:', i
      end if
      i = tstg5()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg5 fail:', i
      end if
      i = tstg6()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg6 fail:', i
      end if
      i = tstg9()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg9 fail:', i
      end if
      i = tstg10()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg10 fail:', i
      end if
      i = tstg11()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg11 fail:', i
      end if
      i = tstg12()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg12 fail:', i
      end if
      i = tstg14()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg14 fail:', i
      end if
      i = tstg15()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg15 fail:', i
      end if
      i = tstg17()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg17 fail:', i
      end if
      i = tstg26()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg26 fail:', i
      end if
      i = tstg28()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg28 fail:', i
      end if
      i = tstg33()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg33 fail:', i
      end if
      i = tstg55()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg55 fail:', i
      end if
      i = tstg59()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg59 fail:', i
      end if
      i = tstg61()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg61 fail:', i
      end if
      i = tstg73()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg73 fail:', i
      end if
      i = tstg74()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg74 fail:', i
      end if
      i = tstg76()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg76 fail:', i
      end if
      i = tstg78()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstg78 fail:', i
      end if
      i = tstp0()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstp0 fail:', i
      end if
      i = tstp5()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstp5 fail:', i
      end if
      i = tstp6()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstp6 fail:', i
      end if
      i = tstp12()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstp12 fail:', i
      end if
      i = tstp13()
      if (i .gt. 0) then
        n = n + 1
        print *, 'tstp13 fail:', i
      end if

      if (n .gt. 0) then
        stop 1
      end if

      stop
      end

*> @endcond SKIP
