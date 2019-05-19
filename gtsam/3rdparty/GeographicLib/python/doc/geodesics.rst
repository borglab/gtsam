Geodesics on an ellipsoid
=========================

Jump to

* :ref:`intro`
* :ref:`additional`
* :ref:`multiple`
* :ref:`background`
* :ref:`references`

.. _intro:

Introduction
------------

Consider a ellipsoid of revolution with equatorial radius *a*, polar
semi-axis *b*, and flattening *f* = (*a* − *b*)/*a* .  Points on
the surface of the ellipsoid are characterized by their latitude φ
and longitude λ.  (Note that latitude here means the
*geographical latitude*, the angle between the normal to the ellipsoid
and the equatorial plane).

The shortest path between two points on the ellipsoid at
(φ\ :sub:`1`, λ\ :sub:`1`) and (φ\ :sub:`2`,
λ\ :sub:`2`) is called the geodesic.  Its length is
*s*\ :sub:`12` and the geodesic from point 1 to point 2 has forward
azimuths α\ :sub:`1` and α\ :sub:`2` at the two end
points.  In this figure, we have λ\ :sub:`12` =
λ\ :sub:`2` − λ\ :sub:`1`.

    .. raw:: html

       <center>
         <img src="https://upload.wikimedia.org/wikipedia/commons/c/cb/Geodesic_problem_on_an_ellipsoid.svg"
              alt="Figure from wikipedia"
              width="250">
       </center>

A geodesic can be extended indefinitely by requiring that any
sufficiently small segment is a shortest path; geodesics are also the
straightest curves on the surface.

Traditionally two geodesic problems are considered:

* the direct problem — given φ\ :sub:`1`,
  λ\ :sub:`1`, α\ :sub:`1`, *s*\ :sub:`12`,
  determine φ\ :sub:`2`, λ\ :sub:`2`, and
  α\ :sub:`2`; this is solved by
  :meth:`Geodesic.Direct <geographiclib.geodesic.Geodesic.Direct>`.

* the inverse problem — given φ\ :sub:`1`,
  λ\ :sub:`1`, φ\ :sub:`2`, λ\ :sub:`2`,
  determine *s*\ :sub:`12`, α\ :sub:`1`, and
  α\ :sub:`2`; this is solved by
  :meth:`Geodesic.Inverse <geographiclib.geodesic.Geodesic.Inverse>`.

.. _additional:

Additional properties
---------------------

The routines also calculate several other quantities of interest

* *S*\ :sub:`12` is the area between the geodesic from point 1 to
  point 2 and the equator; i.e., it is the area, measured
  counter-clockwise, of the quadrilateral with corners
  (φ\ :sub:`1`,λ\ :sub:`1`), (0,λ\ :sub:`1`),
  (0,λ\ :sub:`2`), and
  (φ\ :sub:`2`,λ\ :sub:`2`).  It is given in
  meters\ :sup:`2`.
* *m*\ :sub:`12`, the reduced length of the geodesic is defined such
  that if the initial azimuth is perturbed by *d*\ α\ :sub:`1`
  (radians) then the second point is displaced by *m*\ :sub:`12`
  *d*\ α\ :sub:`1` in the direction perpendicular to the
  geodesic.  *m*\ :sub:`12` is given in meters.  On a curved surface
  the reduced length obeys a symmetry relation, *m*\ :sub:`12` +
  *m*\ :sub:`21` = 0.  On a flat surface, we have *m*\ :sub:`12` =
  *s*\ :sub:`12`.
* *M*\ :sub:`12` and *M*\ :sub:`21` are geodesic scales.  If two
  geodesics are parallel at point 1 and separated by a small distance
  *dt*, then they are separated by a distance *M*\ :sub:`12` *dt* at
  point 2.  *M*\ :sub:`21` is defined similarly (with the geodesics
  being parallel to one another at point 2).  *M*\ :sub:`12` and
  *M*\ :sub:`21` are dimensionless quantities.  On a flat surface,
  we have *M*\ :sub:`12` = *M*\ :sub:`21` = 1.
* σ\ :sub:`12` is the arc length on the auxiliary sphere.
  This is a construct for converting the problem to one in spherical
  trigonometry.  The spherical arc length from one equator crossing to
  the next is always 180°.

If points 1, 2, and 3 lie on a single geodesic, then the following
addition rules hold:

* *s*\ :sub:`13` = *s*\ :sub:`12` + *s*\ :sub:`23`
* σ\ :sub:`13` = σ\ :sub:`12` + σ\ :sub:`23`
* *S*\ :sub:`13` = *S*\ :sub:`12` + *S*\ :sub:`23`
* *m*\ :sub:`13` = *m*\ :sub:`12`\ *M*\ :sub:`23` +
  *m*\ :sub:`23`\ *M*\ :sub:`21`
* *M*\ :sub:`13` = *M*\ :sub:`12`\ *M*\ :sub:`23` −
  (1 − *M*\ :sub:`12`\ *M*\ :sub:`21`)
  *m*\ :sub:`23`/*m*\ :sub:`12`
* *M*\ :sub:`31` = *M*\ :sub:`32`\ *M*\ :sub:`21` −
  (1 − *M*\ :sub:`23`\ *M*\ :sub:`32`)
  *m*\ :sub:`12`/*m*\ :sub:`23`

.. _multiple:

Multiple shortest geodesics
---------------------------

The shortest distance found by solving the inverse problem is
(obviously) uniquely defined.  However, in a few special cases there are
multiple azimuths which yield the same shortest distance.  Here is a
catalog of those cases:

* φ\ :sub:`1` = −φ\ :sub:`2` (with neither point at
  a pole).  If α\ :sub:`1` = α\ :sub:`2`, the geodesic
  is unique.  Otherwise there are two geodesics and the second one is
  obtained by setting [α\ :sub:`1`,α\ :sub:`2`] ←
  [α\ :sub:`2`,α\ :sub:`1`],
  [*M*\ :sub:`12`,\ *M*\ :sub:`21`] ←
  [*M*\ :sub:`21`,\ *M*\ :sub:`12`], *S*\ :sub:`12` ←
  −\ *S*\ :sub:`12`.  (This occurs when the longitude difference
  is near ±180° for oblate ellipsoids.)
* λ\ :sub:`2` = λ\ :sub:`1` ± 180° (with
  neither point at a pole).  If α\ :sub:`1` = 0° or
  ±180°, the geodesic is unique.  Otherwise there are two
  geodesics and the second one is obtained by setting
  [α\ :sub:`1`,α\ :sub:`2`] ←
  [−α\ :sub:`1`,−α\ :sub:`2`],
  *S*\ :sub:`12` ← −\ *S*\ :sub:`12`.  (This occurs when
  φ\ :sub:`2` is near −φ\ :sub:`1` for prolate
  ellipsoids.)
* Points 1 and 2 at opposite poles.  There are infinitely many
  geodesics which can be generated by setting
  [α\ :sub:`1`,α\ :sub:`2`] ←
  [α\ :sub:`1`,α\ :sub:`2`] +
  [δ,−δ], for arbitrary δ.  (For spheres, this
  prescription applies when points 1 and 2 are antipodal.)
* *s*\ :sub:`12` = 0 (coincident points).  There are infinitely many
  geodesics which can be generated by setting
  [α\ :sub:`1`,α\ :sub:`2`] ←
  [α\ :sub:`1`,α\ :sub:`2`] + [δ,δ], for
  arbitrary δ.

.. _background:

Background
----------

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

.. _references:

References
----------

* F. W. Bessel,
  `The calculation of longitude and latitude from geodesic measurements (1825)
  <https://arxiv.org/abs/0908.1824>`_,
  Astron. Nachr. **331**\ (8), 852–861 (2010),
  translated by C. F. F. Karney and R. E. Deakin.
* F. R. Helmert,
  `Mathematical and Physical Theories of Higher Geodesy, Vol 1
  <https://doi.org/10.5281/zenodo.32050>`_,
  (Teubner, Leipzig, 1880), Chaps. 5–7.
* T. Vincenty,
  `Direct and inverse solutions of geodesics on the ellipsoid with
  application of nested equations
  <http://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf>`_,
  Survey Review **23**\ (176), 88–93 (1975).
* J. Danielsen,
  `The area under the geodesic
  <https://doi.org/10.1179/003962689791474267>`_,
  Survey Review **30**\ (232), 61–66 (1989).
* C. F. F. Karney,
  `Algorithms for geodesics
  <https://doi.org/10.1007/s00190-012-0578-z>`_,
  J. Geodesy **87**\ (1) 43–55 (2013);
  `addenda <https://geographiclib.sourceforge.io/geod-addenda.html>`_.
* C. F. F. Karney,
  `Geodesics on an ellipsoid of revolution
  <https://arxiv.org/abs/1102.1215v1>`_,
  Feb. 2011;
  `errata
  <https://geographiclib.sourceforge.io/geod-addenda.html#geod-errata>`_.
* `A geodesic bibliography
  <https://geographiclib.sourceforge.io/geodesic-papers/biblio.html>`_.
* The wikipedia page,
  `Geodesics on an ellipsoid
  <https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid>`_.
