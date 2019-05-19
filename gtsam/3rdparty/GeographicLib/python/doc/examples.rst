Examples
========

Initializing
------------

The following examples all assume that the following commands have been
carried out:

  >>> from geographiclib.geodesic import Geodesic
  >>> import math
  >>> geod = Geodesic.WGS84  # define the WGS84 ellipsoid

You can determine the ellipsoid parameters with the *a* and *f* member
variables, for example,

  >>> geod.a, 1/geod.f
  (6378137.0, 298.257223563)

If you need to use a different ellipsoid, construct one by, for example

  >>> geod = Geodesic(6378388, 1/297.0) # the international ellipsoid

Basic geodesic calculations
---------------------------

The distance from Wellington, NZ (41.32S, 174.81E) to Salamanca, Spain
(40.96N, 5.50W) using :meth:`~geographiclib.geodesic.Geodesic.Inverse`:

  >>> g = geod.Inverse(-41.32, 174.81, 40.96, -5.50)
  >>> print "The distance is {:.3f} m.".format(g['s12'])
  The distance is 19959679.267 m.

The point the point 20000 km SW of Perth, Australia (32.06S, 115.74E)
using :meth:`~geographiclib.geodesic.Geodesic.Direct`:

  >>> g = geod.Direct(-32.06, 115.74, 225, 20000e3)
  >>> print "The position is ({:.8f}, {:.8f}).".format(g['lat2'],g['lon2'])
  The position is (32.11195529, -63.95925278).

The area between the geodesic from JFK Airport (40.6N, 73.8W) to LHR
Airport (51.6N, 0.5W) and the equator. This is an example of setting the
the :ref:`output mask <outmask>` parameter.

  >>> g = geod.Inverse(40.6, -73.8, 51.6, -0.5, Geodesic.AREA)
  >>> print "The area is {:.1f}  m^2".format(g['S12'])
  The area is 40041368848742.5  m^2

Computing waypoints
-------------------

Consider the geodesic between Beijing Airport (40.1N, 116.6E) and San
Fransisco Airport (37.6N, 122.4W). Compute waypoints and azimuths at
intervals of 1000 km using
:meth:`Geodesic.Line <geographiclib.geodesic.Geodesic.Line>` and
:meth:`GeodesicLine.Position
<geographiclib.geodesicline.GeodesicLine.Position>`:

  >>> l = geod.InverseLine(40.1, 116.6, 37.6, -122.4)
  >>> ds = 1000e3; n = int(math.ceil(l.s13 / ds))
  >>> for i in range(n + 1):
  ...   if i == 0:
  ...     print "distance latitude longitude azimuth"
  ...   s = min(ds * i, l.s13)
  ...   g = l.Position(s, Geodesic.STANDARD | Geodesic.LONG_UNROLL)
  ...   print "{:.0f} {:.5f} {:.5f} {:.5f}".format(
  ...     g['s12'], g['lat2'], g['lon2'], g['azi2'])
  ...
  distance latitude longitude azimuth
  0 40.10000 116.60000 42.91642
  1000000 46.37321 125.44903 48.99365
  2000000 51.78786 136.40751 57.29433
  3000000 55.92437 149.93825 68.24573
  4000000 58.27452 165.90776 81.68242
  5000000 58.43499 183.03167 96.29014
  6000000 56.37430 199.26948 109.99924
  7000000 52.45769 213.17327 121.33210
  8000000 47.19436 224.47209 129.98619
  9000000 41.02145 233.58294 136.34359
  9513998 37.60000 237.60000 138.89027

The inclusion of Geodesic.LONG_UNROLL in the call to
GeodesicLine.Position ensures that the longitude does not jump on
crossing the international dateline.

If the purpose of computing the waypoints is to plot a smooth geodesic,
then it's not important that they be exactly equally spaced. In this
case, it's faster to parameterize the line in terms of the spherical arc
length with :meth:`GeodesicLine.ArcPosition
<geographiclib.geodesicline.GeodesicLine.ArcPosition>`. Here the
spacing is about 1° of arc which means that the distance between the
waypoints will be about 60 NM.

  >>> l = geod.InverseLine(40.1, 116.6, 37.6, -122.4,
  ...               Geodesic.LATITUDE | Geodesic.LONGITUDE)
  >>> da = 1; n = int(math.ceil(l.a13 / da)); da = l.a13 / n
  >>> for i in range(n + 1):
  ...   if i == 0:
  ...     print "latitude longitude"
  ...   a = da * i
  ...   g = l.ArcPosition(a, Geodesic.LATITUDE |
  ...                     Geodesic.LONGITUDE | Geodesic.LONG_UNROLL)
  ...   print "{:.5f} {:.5f}".format(g['lat2'], g['lon2'])
  ...
  latitude longitude
  40.10000 116.60000
  40.82573 117.49243
  41.54435 118.40447
  42.25551 119.33686
  42.95886 120.29036
  43.65403 121.26575
  44.34062 122.26380
  ...
  39.82385 235.05331
  39.08884 235.91990
  38.34746 236.76857
  37.60000 237.60000

The variation in the distance between these waypoints is on the order of
1/*f*.

Measuring areas
---------------

Measure the area of Antarctica using
:meth:`Geodesic.Polygon <geographiclib.geodesic.Geodesic.Polygon>` and the
:class:`~geographiclib.polygonarea.PolygonArea` class:

  >>> p = geod.Polygon()
  >>> antarctica = [
  ...   [-63.1, -58], [-72.9, -74], [-71.9,-102], [-74.9,-102], [-74.3,-131],
  ...   [-77.5,-163], [-77.4, 163], [-71.7, 172], [-65.9, 140], [-65.7, 113],
  ...   [-66.6,  88], [-66.9,  59], [-69.8,  25], [-70.0,  -4], [-71.0, -14],
  ...   [-77.3, -33], [-77.9, -46], [-74.7, -61]
  ... ]
  >>> for pnt in antarctica:
  ...   p.AddPoint(pnt[0], pnt[1])
  ...
  >>> num, perim, area = p.Compute()
  >>> print "Perimeter/area of Antarctica are {:.3f} m / {:.1f} m^2".format(
  ...   perim, area)
  Perimeter/area of Antarctica are 16831067.893 m / 13662703680020.1 m^2
