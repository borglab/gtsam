Jump to
* [Introduction](#intro)
* [Additional properties](#additional)
* [Multiple shortest geodesics](#multiple)
* [Background](#background)
* [References](#references)

### <a name="intro"></a>Introduction

Consider a ellipsoid of revolution with equatorial radius *a*, polar
semi-axis *b*, and flattening *f* = (*a* &minus; *b*)/*a* .  Points on
the surface of the ellipsoid are characterized by their latitude &phi;
and longitude &lambda;.  (Note that latitude here means the
*geographical latitude*, the angle between the normal to the ellipsoid
and the equatorial plane).

The shortest path between two points on the ellipsoid at
(&phi;<sub>1</sub>, &lambda;<sub>1</sub>) and (&phi;<sub>2</sub>,
&lambda;<sub>2</sub>) is called the geodesic.  Its length is
*s*<sub>12</sub> and the geodesic from point 1 to point 2 has forward
azimuths &alpha;<sub>1</sub> and &alpha;<sub>2</sub> at the two end
points.  In this figure, we have &lambda;<sub>12</sub> =
&lambda;<sub>2</sub> &minus; &lambda;<sub>1</sub>.
<center>
<img src="https://upload.wikimedia.org/wikipedia/commons/c/cb/Geodesic_problem_on_an_ellipsoid.svg" width="250">
</center>
A geodesic can be extended indefinitely by requiring that any
sufficiently small segment is a shortest path; geodesics are also the
straightest curves on the surface.

Traditionally two geodesic problems are considered:
* the direct problem &mdash; given &phi;<sub>1</sub>,
  &lambda;<sub>1</sub>, &alpha;<sub>1</sub>, *s*<sub>12</sub>,
  determine &phi;<sub>2</sub>, &lambda;<sub>2</sub>, and
  &alpha;<sub>2</sub>; this is solved by
  {@link module:GeographicLib/Geodesic.Geodesic#Direct Geodesic.Direct}.

* the inverse problem &mdash; given &phi;<sub>1</sub>,
  &lambda;<sub>1</sub>, &phi;<sub>2</sub>, &lambda;<sub>2</sub>,
  determine *s*<sub>12</sub>, &alpha;<sub>1</sub>, and
  &alpha;<sub>2</sub>; this is solved by
  {@link module:GeographicLib/Geodesic.Geodesic#Inverse Geodesic.Inverse}.

### <a name="additional"></a>Additional properties

The routines also calculate several other quantities of interest
* *S*<sub>12</sub> is the area between the geodesic from point 1 to
  point 2 and the equator; i.e., it is the area, measured
  counter-clockwise, of the quadrilateral with corners
  (&phi;<sub>1</sub>,&lambda;<sub>1</sub>), (0,&lambda;<sub>1</sub>),
  (0,&lambda;<sub>2</sub>), and
  (&phi;<sub>2</sub>,&lambda;<sub>2</sub>).  It is given in
  meters<sup>2</sup>.
* *m*<sub>12</sub>, the reduced length of the geodesic is defined such
  that if the initial azimuth is perturbed by *d*&alpha;<sub>1</sub>
  (radians) then the second point is displaced by *m*<sub>12</sub>
  *d*&alpha;<sub>1</sub> in the direction perpendicular to the
  geodesic.  *m*<sub>12</sub> is given in meters.  On a curved surface
  the reduced length obeys a symmetry relation, *m*<sub>12</sub> +
  *m*<sub>21</sub> = 0.  On a flat surface, we have *m*<sub>12</sub> =
  *s*<sub>12</sub>.
* *M*<sub>12</sub> and *M*<sub>21</sub> are geodesic scales.  If two
  geodesics are parallel at point 1 and separated by a small distance
  *dt*, then they are separated by a distance *M*<sub>12</sub> *dt* at
  point 2.  *M*<sub>21</sub> is defined similarly (with the geodesics
  being parallel to one another at point 2).  *M*<sub>12</sub> and
  *M*<sub>21</sub> are dimensionless quantities.  On a flat surface,
  we have *M*<sub>12</sub> = *M*<sub>21</sub> = 1.
* &sigma;<sub>12</sub> is the arc length on the auxiliary sphere.
  This is a construct for converting the problem to one in spherical
  trigonometry.  The spherical arc length from one equator crossing to
  the next is always 180&deg;.

If points 1, 2, and 3 lie on a single geodesic, then the following
addition rules hold:
* *s*<sub>13</sub> = *s*<sub>12</sub> + *s*<sub>23</sub>
* &sigma;<sub>13</sub> = &sigma;<sub>12</sub> + &sigma;<sub>23</sub>
* *S*<sub>13</sub> = *S*<sub>12</sub> + *S*<sub>23</sub>
* *m*<sub>13</sub> = *m*<sub>12</sub>*M*<sub>23</sub> +
  *m*<sub>23</sub>*M*<sub>21</sub>
* *M*<sub>13</sub> = *M*<sub>12</sub>*M*<sub>23</sub> &minus;
  (1 &minus; *M*<sub>12</sub>*M*<sub>21</sub>)
  *m*<sub>23</sub>/*m*<sub>12</sub>
* *M*<sub>31</sub> = *M*<sub>32</sub>*M*<sub>21</sub> &minus;
  (1 &minus; *M*<sub>23</sub>*M*<sub>32</sub>)
  *m*<sub>12</sub>/*m*<sub>23</sub>

### <a name="multiple"></a>Multiple shortest geodesics

The shortest distance found by solving the inverse problem is
(obviously) uniquely defined.  However, in a few special cases there are
multiple azimuths which yield the same shortest distance.  Here is a
catalog of those cases:
* &phi;<sub>1</sub> = &minus;&phi;<sub>2</sub> (with neither point at
  a pole).  If &alpha;<sub>1</sub> = &alpha;<sub>2</sub>, the geodesic
  is unique.  Otherwise there are two geodesics and the second one is
  obtained by setting [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] &larr;
  [&alpha;<sub>2</sub>,&alpha;<sub>1</sub>],
  [*M*<sub>12</sub>,*M*<sub>21</sub>] &larr;
  [*M*<sub>21</sub>,*M*<sub>12</sub>], *S*<sub>12</sub> &larr;
  &minus;*S*<sub>12</sub>.  (This occurs when the longitude difference
  is near &plusmn;180&deg; for oblate ellipsoids.)
* &lambda;<sub>2</sub> = &lambda;<sub>1</sub> &plusmn; 180&deg; (with
  neither point at a pole).  If &alpha;<sub>1</sub> = 0&deg; or
  &plusmn;180&deg;, the geodesic is unique.  Otherwise there are two
  geodesics and the second one is obtained by setting
  [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] &larr;
  [&minus;&alpha;<sub>1</sub>,&minus;&alpha;<sub>2</sub>],
  *S*<sub>12</sub> &larr; &minus;*S*<sub>12</sub>.  (This occurs when
  &phi;<sub>2</sub> is near &minus;&phi;<sub>1</sub> for prolate
  ellipsoids.)
* Points 1 and 2 at opposite poles.  There are infinitely many
  geodesics which can be generated by setting
  [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] &larr;
  [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] +
  [&delta;,&minus;&delta;], for arbitrary &delta;.  (For spheres, this
  prescription applies when points 1 and 2 are antipodal.)
* *s*<sub>12</sub> = 0 (coincident points).  There are infinitely many
  geodesics which can be generated by setting
  [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] &larr;
  [&alpha;<sub>1</sub>,&alpha;<sub>2</sub>] + [&delta;,&delta;], for
  arbitrary &delta;.

### <a name="background"></a>Background

The algorithms implemented by this package are given in Karney (2013)
and are based on Bessel (1825) and Helmert (1880); the algorithm for
areas is based on Danielsen (1989).  These improve on the work of
Vincenty (1975) in the following respects:
* The results are accurate to round-off for terrestrial ellipsoids (the
  error in the distance is less then 15 nanometers, compared to 0.1 mm
  for Vincenty).
* The solution of the inverse problem is always found.  (Vincenty's
  method fails to converge for nearly antipodal points.)
* The routines calculate differential and integral properties of a
  geodesic.  This allows, for example, the area of a geodesic polygon to
  be computed.

### <a name="references"></a>References

* F. W. Bessel,
  {@link https://arxiv.org/abs/0908.1824 The calculation of longitude and
  latitude from geodesic measurements (1825)},
  Astron. Nachr. **331**(8), 852&ndash;861 (2010),
  translated by C. F. F. Karney and R. E. Deakin.
* F. R. Helmert,
  {@link https://doi.org/10.5281/zenodo.32050
  Mathematical and Physical Theories of Higher Geodesy, Vol 1},
  (Teubner, Leipzig, 1880), Chaps. 5&ndash;7.
* T. Vincenty,
  {@link http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
  Direct and inverse solutions of geodesics on the ellipsoid with
  application of nested equations},
  Survey Review **23**(176), 88&ndash;93 (1975).
* J. Danielsen,
  {@link https://doi.org/10.1179/003962689791474267 The area under
  the geodesic}, Survey Review **30**(232), 61&ndash;66 (1989).
* C. F. F. Karney,
  {@link https://doi.org/10.1007/s00190-012-0578-z
  Algorithms for geodesics}, J. Geodesy **87**(1) 43&ndash;55 (2013);
  {@link https://geographiclib.sourceforge.io/geod-addenda.html addenda}.
* C. F. F. Karney,
  {@https://arxiv.org/abs/1102.1215v1
  Geodesics on an ellipsoid of revolution},
  Feb. 2011;
  {@link https://geographiclib.sourceforge.io/geod-addenda.html#geod-errata
  errata}.
* {@link https://geographiclib.sourceforge.io/geodesic-papers/biblio.html
  A geodesic bibliography}.
* The wikipedia page,
  {@link https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid
  Geodesics on an ellipsoid}.
