The library interface
=====================

Jump to

* :ref:`units`
* :ref:`dict`
* :ref:`outmask`
* :ref:`restrictions`

.. _units:

The units
---------

All angles (latitude, longitude, azimuth, arc length) are measured in
degrees with latitudes increasing northwards, longitudes increasing
eastwards, and azimuths measured clockwise from north. For a point at a
pole, the azimuth is defined by keeping the longitude fixed, writing φ =
±(90° − ε), and taking the limit ε → 0+

.. _dict:

Geodesic dictionary
-------------------

The results returned by
:meth:`Geodesic.Direct <geographiclib.geodesic.Geodesic.Direct>`,
:meth:`Geodesic.Inverse <geographiclib.geodesic.Geodesic.Inverse>`,
:meth:`GeodesicLine.Position <geographiclib.geodesicline.GeodesicLine.Position>`,
etc., return a dictionary with some of the following 12 fields set:

* *lat1* = φ\ :sub:`1`, latitude of point 1 (degrees)
* *lon1* = λ\ :sub:`1`, longitude of point 1 (degrees)
* *azi1* = α\ :sub:`1`, azimuth of line at point 1 (degrees)
* *lat2* = φ\ :sub:`2`, latitude of point 2 (degrees)
* *lon2* = λ\ :sub:`2`, longitude of point 2 (degrees)
* *azi2* = α\ :sub:`2`, (forward) azimuth of line at point 2 (degrees)
* *s12* = *s*\ :sub:`12`, distance from 1 to 2 (meters)
* *a12* = σ\ :sub:`12`, arc length on auxiliary sphere from 1 to 2 (degrees)
* *m12* = *m*\ :sub:`12`, reduced length of geodesic (meters)
* *M12* = *M*\ :sub:`12`, geodesic scale at 2 relative to 1 (dimensionless)
* *M21* = *M*\ :sub:`21`, geodesic scale at 1 relative to 2 (dimensionless)
* *S12* = *S*\ :sub:`12`, area between geodesic and equator (meters\ :sup:`2`)

.. _outmask:

*outmask* and *caps*
--------------------

By default, the geodesic routines return the 7 basic quantities: *lat1*,
*lon1*, *azi1*, *lat2*, *lon2*, *azi2*, *s12*, together with the arc
length *a12*.  The optional output mask parameter, *outmask*, can be
used to tailor which quantities to calculate.  In addition, when a
:class:`~geographiclib.geodesicline.GeodesicLine` is constructed it can
be provided with the optional capabilities parameter, *caps*, which
specifies what quantities can be returned from the resulting object.

Both *outmask* and *caps* are obtained by or'ing together the following
values

* :data:`~geographiclib.geodesic.Geodesic.EMPTY`, no capabilities, no output
* :data:`~geographiclib.geodesic.Geodesic.LATITUDE`, compute latitude, *lat2*
* :data:`~geographiclib.geodesic.Geodesic.LONGITUDE`,
  compute longitude, *lon2*
* :data:`~geographiclib.geodesic.Geodesic.AZIMUTH`,
  compute azimuths, *azi1* and *azi2*
* :data:`~geographiclib.geodesic.Geodesic.DISTANCE`, compute distance, *s12*
* :data:`~geographiclib.geodesic.Geodesic.STANDARD`, all of the above
* :data:`~geographiclib.geodesic.Geodesic.DISTANCE_IN`,
  allow *s12* to be used as input in the direct problem
* :data:`~geographiclib.geodesic.Geodesic.REDUCEDLENGTH`,
  compute reduced length, *m12*
* :data:`~geographiclib.geodesic.Geodesic.GEODESICSCALE`,
  compute geodesic scales, *M12* and *M21*
* :data:`~geographiclib.geodesic.Geodesic.AREA`, compute area, *S12*
* :data:`~geographiclib.geodesic.Geodesic.ALL`, all of the above;
* :data:`~geographiclib.geodesic.Geodesic.LONG_UNROLL`, unroll longitudes

DISTANCE_IN is a capability provided to the GeodesicLine constructor. It
allows the position on the line to specified in terms of
distance.  (Without this, the position can only be specified in terms of
the arc length.)  This only makes sense in the *caps* parameter.

LONG_UNROLL controls the treatment of longitude. If it is not set then
the *lon1* and *lon2* fields are both reduced to the range [−180°,
180°). If it is set, then *lon1* is as given in the function call and
(*lon2* − *lon1*) determines how many times and in what sense the
geodesic has encircled the ellipsoid.  This only makes sense in the
*outmask* parameter.

Note that *a12* is always included in the result.

.. _restrictions:

Restrictions on the parameters
------------------------------

* Latitudes must lie in [−90°, 90°]. Latitudes outside this range are
  replaced by NaNs.
* The distance *s12* is unrestricted. This allows geodesics to wrap
  around the ellipsoid. Such geodesics are no longer shortest
  paths. However they retain the property that they are the straightest
  curves on the surface.
* Similarly, the spherical arc length *a12* is unrestricted.
* Longitudes and azimuths are unrestricted; internally these are
  exactly reduced to the range [−180°, 180°); but see also the
  LONG_UNROLL bit.
* The equatorial radius *a* and the polar semi-axis *b* must both be
  positive and finite (this implies that −∞ < *f* < 1).
* The flattening *f* should satisfy *f* ∈ [−1/50,1/50] in order to retain
  full accuracy. This condition holds for most applications in geodesy.

Reasonably accurate results can be obtained for −0.2 ≤ *f* ≤ 0.2. Here
is a table of the approximate maximum error (expressed as a distance)
for an ellipsoid with the same equatorial radius as the WGS84 ellipsoid
and different values of the flattening.

======== =======
abs(*f*) error
-------- -------
0.003    15 nm
0.01     25 nm
0.02     30 nm
0.05     10 μm
0.1      1.5 mm
0.2      300 mm
======== =======

Here 1 nm = 1 nanometer = 10\ :sup:`−9` m (*not* 1 nautical mile!)
