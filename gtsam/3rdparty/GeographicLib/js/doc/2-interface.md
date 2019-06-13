Jump to
* [The units](#units)
* [The results](#results)
* [The *outmask* and *caps* parameters](#outmask)
* [Restrictions on the parameters](#restrict)

### <a name="units"></a>The units

All angles (latitude, longitude, azimuth, arc length) are measured in
degrees with latitudes increasing northwards, longitudes increasing
eastwards, and azimuths measured clockwise from north.  For a point at a
pole, the azimuth is defined by keeping the longitude fixed, writing
&phi; = &plusmn;(90&deg; &minus; &epsilon;), and taking the limit
&epsilon; &rarr; 0+.

### <a name="results"></a>The results

The results returned by
{@link module:GeographicLib/Geodesic.Geodesic#Inverse Geodesic.Direct},
{@link module:GeographicLib/Geodesic.Geodesic#Inverse Geodesic.Inverse},
{@link module:GeographicLib/GeodesicLine.GeodesicLine#Position
GeodesicLine.Position}, etc., return an object with
(some) of the following 12 fields set:
* *lat1* = &phi;<sub>1</sub>, latitude of point 1 (degrees)
* *lon1* = &lambda;<sub>1</sub>, longitude of point 1 (degrees)
* *azi1* = &alpha;<sub>1</sub>, azimuth of line at point 1 (degrees)
* *lat2* = &phi;<sub>2</sub>, latitude of point 2 (degrees)
* *lon2* = &lambda;<sub>2</sub>, longitude of point 2 (degrees)
* *azi2* = &alpha;<sub>2</sub>, (forward) azimuth of line at point 2 (degrees)
* *s12* = *s*<sub>12</sub>, distance from 1 to 2 (meters)
* *a12* = &sigma;<sub>12</sub>, arc length on auxiliary sphere from 1 to 2
  (degrees)
* *m12* = *m*<sub>12</sub>, reduced length of geodesic (meters)
* *M12* = *M*<sub>12</sub>, geodesic scale at 2 relative to 1 (dimensionless)
* *M21* = *M*<sub>21</sub>, geodesic scale at 1 relative to 2 (dimensionless)
* *S12* = *S*<sub>12</sub>, area between geodesic and equator
  (meters<sup>2</sup>)

The input parameters together with *a12* are always included in the
object.  Azimuths are reduced to the range [&minus;180&deg;, 180&deg;].
See {@tutorial 1-geodesics} for the definitions of these quantities.

### <a name="outmask"></a>The *outmask* and *caps* parameters

By default, the geodesic routines return the 7 basic quantities: *lat1*,
*lon1*, *azi1*, *lat2*, *lon2*, *azi2*, *s12*, together with the arc
length *a12*.  The optional output mask parameter, *outmask*, can be
used to tailor which quantities to calculate.  In addition, when a
{@link module:GeographicLib/GeodesicLine.GeodesicLine GeodesicLine} is
constructed it can be provided with the optional capabilities parameter,
*caps*, which specifies what quantities can be returned from the
resulting object.

Both *outmask* and *caps* are obtained by or'ing together the following
values
* Geodesic.NONE, no capabilities, no output;
* Geodesic.ARC, compute arc length, *a12*; this is always implicitly set;
* Geodesic.LATITUDE, compute latitude, *lat2*;
* Geodesic.LONGITUDE, compute longitude, *lon2*;
* Geodesic.AZIMUTH, compute azimuths, *azi1* and *azi2*;
* Geodesic.DISTANCE, compute distance, *s12*;
* Geodesic.STANDARD, all of the above;
* Geodesic.DISTANCE_IN, allow *s12* to be used as input in the direct problem;
* Geodesic.REDUCEDLENGTH, compute reduced length, *m12*;
* Geodesic.GEODESICSCALE, compute geodesic scales, *M12* and *M21*;
* Geodesic.AREA, compute area, *S12*;
* Geodesic.ALL, all of the above;
* Geodesic.LONG_UNROLL, unroll longitudes.

Geodesic.DISTANCE_IN is a capability provided to the
{@link module:GeographicLib/GeodesicLine.GeodesicLine GeodesicLine}
constructor.  It allows the position on the line to specified in terms
of distance.  (Without this, the position can only be specified in terms
of the arc length.)  This only makes sense in the *caps* parameter.

Geodesic.LONG_UNROLL controls the treatment of longitude.  If it is not
set then the *lon1* and *lon2* fields are both reduced to the range
[&minus;180&deg;, 180&deg;].  If it is set, then *lon1* is as given in
the function call and (*lon2* &minus; *lon1*) determines how many times
and in what sense the geodesic has encircled the ellipsoid.  This only
makes sense in the *outmask* parameter.

Note that *a12* is always included in the result.

### <a name="restrict"></a>Restrictions on the parameters

* Latitudes must lie in [&minus;90&deg;, 90&deg;].  Latitudes outside
  this range are replaced by NaNs.
* The distance *s12* is unrestricted.  This allows geodesics to wrap
  around the ellipsoid.  Such geodesics are no longer shortest paths.
  However they retain the property that they are the straightest curves
  on the surface.
* Similarly, the spherical arc length *a12* is unrestricted.
* Longitudes and azimuths are unrestricted; internally these are exactly
  reduced to the range [&minus;180&deg;, 180&deg;]; but see also the
  LONG_UNROLL bit.
* The equatorial radius *a* and the polar semi-axis *b* must both be
  positive and finite (this implies that &minus;&infin; &lt; *f* &lt; 1).
* The flattening *f* should satisfy *f* &isin; [&minus;1/50,1/50] in
  order to retain full accuracy.  This condition holds for most
  applications in geodesy.

Reasonably accurate results can be obtained for &minus;0.2 &le; *f* &le;
0.2.  Here is a table of the approximate maximum error (expressed as a
distance) for an ellipsoid with the same equatorial radius as the WGS84
ellipsoid and different values of the flattening.

  | abs(f) | error
  |:-------|------:
  | 0.003  |  15 nm
  | 0.01   |  25 nm
  | 0.02   |  30 nm
  | 0.05   |  10 &mu;m
  | 0.1    | 1.5 mm
  | 0.2    | 300 mm

Here 1 nm = 1 nanometer = 10<sup>&minus;9</sup> m (*not* 1 nautical mile!)
