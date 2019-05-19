/*
 * PolygonArea.js
 * Transcription of PolygonArea.[ch]pp into JavaScript.
 *
 * See the documentation for the C++ class.  The conversion is a literal
 * conversion from C++.
 *
 * The algorithms are derived in
 *
 *    Charles F. F. Karney,
 *    Algorithms for geodesics, J. Geodesy 87, 43-55 (2013);
 *    https://doi.org/10.1007/s00190-012-0578-z
 *    Addenda: https://geographiclib.sourceforge.io/geod-addenda.html
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 */

// Load AFTER GeographicLib/Math.js and GeographicLib/Geodesic.js

(function(
  /**
   * @exports GeographicLib/PolygonArea
   * @description Compute the area of geodesic polygons via the
   *   {@link module:GeographicLib/PolygonArea.PolygonArea PolygonArea}
   *   class.
   */
  p, g, m, a) {

  var transit, transitdirect;
  transit = function(lon1, lon2) {
    // Return 1 or -1 if crossing prime meridian in east or west direction.
    // Otherwise return zero.
    var lon12, cross;
    // Compute lon12 the same way as Geodesic::Inverse.
    lon1 = m.AngNormalize(lon1);
    lon2 = m.AngNormalize(lon2);
    lon12 = m.AngDiff(lon1, lon2).s;
    cross = lon1 <= 0 && lon2 > 0 && lon12 > 0 ? 1 :
      (lon2 <= 0 && lon1 > 0 && lon12 < 0 ? -1 : 0);
    return cross;
  };

  // an alternate version of transit to deal with longitudes in the direct
  // problem.
  transitdirect = function(lon1, lon2) {
    // We want to compute exactly
    //   int(floor(lon2 / 360)) - int(floor(lon1 / 360))
    // Since we only need the parity of the result we can use std::remquo but
    // this is buggy with g++ 4.8.3 and requires C++11.  So instead we do
    lon1 = lon1 % 720.0; lon2 = lon2 % 720.0;
    return ( ((lon2 >= 0 && lon2 < 360) || lon2 < -360 ? 0 : 1) -
             ((lon1 >= 0 && lon1 < 360) || lon1 < -360 ? 0 : 1) );
  };

  /**
   * @class
   * @property {number} a the equatorial radius (meters).
   * @property {number} f the flattening.
   * @property {bool} polyline whether the PolygonArea object describes a
   *   polyline or a polygon.
   * @property {number} num the number of vertices so far.
   * @property {number} lat the current latitude (degrees).
   * @property {number} lon the current longitude (degrees).
   * @summary Initialize a PolygonArea object.
   * @classdesc Computes the area and perimeter of a geodesic polygon.
   *   This object is usually instantiated by
   *   {@link module:GeographicLib/Geodesic.Geodesic#Polygon Geodesic.Polygon}.
   * @param {object} geod a {@link module:GeographicLib/Geodesic.Geodesic
   *   Geodesic} object.
   * @param {bool} [polyline = false] if true the new PolygonArea object
   *   describes a polyline instead of a polygon.
   */
  p.PolygonArea = function(geod, polyline) {
    this._geod = geod;
    this.a = this._geod.a;
    this.f = this._geod.f;
    this._area0 = 4 * Math.PI * geod._c2;
    this.polyline = !polyline ? false : polyline;
    this._mask = g.LATITUDE | g.LONGITUDE | g.DISTANCE |
          (this.polyline ? g.NONE : g.AREA | g.LONG_UNROLL);
    if (!this.polyline)
      this._areasum = new a.Accumulator(0);
    this._perimetersum = new a.Accumulator(0);
    this.Clear();
  };

  /**
   * @summary Clear the PolygonArea object, setting the number of vertices to
   *   0.
   */
  p.PolygonArea.prototype.Clear = function() {
    this.num = 0;
    this._crossings = 0;
    if (!this.polyline)
      this._areasum.Set(0);
    this._perimetersum.Set(0);
    this._lat0 = this._lon0 = this.lat = this.lon = Number.NaN;
  };

  /**
   * @summary Add the next vertex to the polygon.
   * @param {number} lat the latitude of the point (degrees).
   * @param {number} lon the longitude of the point (degrees).
   * @description This adds an edge from the current vertex to the new vertex.
   */
  p.PolygonArea.prototype.AddPoint = function(lat, lon) {
    var t;
    if (this.num === 0) {
      this._lat0 = this.lat = lat;
      this._lon0 = this.lon = lon;
    } else {
      t = this._geod.Inverse(this.lat, this.lon, lat, lon, this._mask);
      this._perimetersum.Add(t.s12);
      if (!this.polyline) {
        this._areasum.Add(t.S12);
        this._crossings += transit(this.lon, lon);
      }
      this.lat = lat;
      this.lon = lon;
    }
    ++this.num;
  };

  /**
   * @summary Add the next edge to the polygon.
   * @param {number} azi the azimuth at the current the point (degrees).
   * @param {number} s the length of the edge (meters).
   * @description This specifies the new vertex in terms of the edge from the
   *   current vertex.
   */
  p.PolygonArea.prototype.AddEdge = function(azi, s) {
    var t;
    if (this.num) {
      t = this._geod.Direct(this.lat, this.lon, azi, s, this._mask);
      this._perimetersum.Add(s);
      if (!this.polyline) {
        this._areasum.Add(t.S12);
        this._crossings += transitdirect(this.lon, t.lon2);
      }
      this.lat = t.lat2;
      this.lon = t.lon2;
    }
    ++this.num;
  };

  /**
   * @summary Compute the perimeter and area of the polygon.
   * @param {bool} reverse if true then clockwise (instead of
   *   counter-clockwise) traversal counts as a positive area.
   * @param {bool} sign if true then return a signed result for the area if the
   *   polygon is traversed in the "wrong" direction instead of returning the
   *   area for the rest of the earth.
   * @returns {object} r where r.number is the number of vertices, r.perimeter
   *   is the perimeter (meters), and r.area (only returned if polyline is
   *   false) is the area (meters<sup>2</sup>).
   * @description If the object is a polygon (and not a polygon), the perimeter
   *   includes the length of a final edge connecting the current point to the
   *   initial point.  If the object is a polyline, then area is nan.  More
   *   points can be added to the polygon after this call.
   */
  p.PolygonArea.prototype.Compute = function(reverse, sign) {
    var vals = {number: this.num}, t, tempsum, crossings;
    if (this.num < 2) {
      vals.perimeter = 0;
      if (!this.polyline)
        vals.area = 0;
      return vals;
    }
    if (this.polyline) {
      vals.perimeter = this._perimetersum.Sum();
      return vals;
    }
    t = this._geod.Inverse(this.lat, this.lon, this._lat0, this._lon0,
                           this._mask);
    vals.perimeter = this._perimetersum.Sum(t.s12);
    tempsum = new a.Accumulator(this._areasum);
    tempsum.Add(t.S12);
    crossings = this._crossings + transit(this.lon, this._lon0);
    if (crossings & 1)
      tempsum.Add( (tempsum.Sum() < 0 ? 1 : -1) * this._area0/2 );
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum.Negate();
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum.Sum() > this._area0/2)
        tempsum.Add( -this._area0 );
      else if (tempsum.Sum() <= -this._area0/2)
        tempsum.Add( +this._area0 );
    } else {
      if (tempsum.Sum() >= this._area0)
        tempsum.Add( -this._area0 );
      else if (tempsum < 0)
        tempsum.Add( -this._area0 );
    }
    vals.area = tempsum.Sum();
    return vals;
  };

  /**
   * @summary Compute the perimeter and area of the polygon with a tentative
   *   new vertex.
   * @param {number} lat the latitude of the point (degrees).
   * @param {number} lon the longitude of the point (degrees).
   * @param {bool} reverse if true then clockwise (instead of
   *   counter-clockwise) traversal counts as a positive area.
   * @param {bool} sign if true then return a signed result for the area if the
   *   polygon is traversed in the "wrong" direction instead of returning the
   * @returns {object} r where r.number is the number of vertices, r.perimeter
   *   is the perimeter (meters), and r.area (only returned if polyline is
   *   false) is the area (meters<sup>2</sup>).
   * @description A new vertex is *not* added to the polygon.
   */
  p.PolygonArea.prototype.TestPoint = function(lat, lon, reverse, sign) {
    var vals = {number: this.num + 1}, t, tempsum, crossings, i;
    if (this.num === 0) {
      vals.perimeter = 0;
      if (!this.polyline)
        vals.area = 0;
      return vals;
    }
    vals.perimeter = this._perimetersum.Sum();
    tempsum = this.polyline ? 0 : this._areasum.Sum();
    crossings = this._crossings;
    for (i = 0; i < (this.polyline ? 1 : 2); ++i) {
      t = this._geod.Inverse(
       i === 0 ? this.lat : lat, i === 0 ? this.lon : lon,
       i !== 0 ? this._lat0 : lat, i !== 0 ? this._lon0 : lon,
       this._mask);
      vals.perimeter += t.s12;
      if (!this.polyline) {
        tempsum += t.S12;
        crossings += transit(i === 0 ? this.lon : lon,
                               i !== 0 ? this._lon0 : lon);
      }
    }

    if (this.polyline)
      return vals;

    if (crossings & 1)
      tempsum += (tempsum < 0 ? 1 : -1) * this._area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum *= -1;
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum > this._area0/2)
        tempsum -= this._area0;
      else if (tempsum <= -this._area0/2)
        tempsum += this._area0;
    } else {
      if (tempsum >= this._area0)
        tempsum -= this._area0;
      else if (tempsum < 0)
        tempsum += this._area0;
    }
    vals.area = tempsum;
    return vals;
  };

  /**
   * @summary Compute the perimeter and area of the polygon with a tentative
   *   new edge.
   * @param {number} azi the azimuth of the edge (degrees).
   * @param {number} s the length of the edge (meters).
   * @param {bool} reverse if true then clockwise (instead of
   *   counter-clockwise) traversal counts as a positive area.
   * @param {bool} sign if true then return a signed result for the area if the
   *   polygon is traversed in the "wrong" direction instead of returning the
   * @returns {object} r where r.number is the number of vertices, r.perimeter
   *   is the perimeter (meters), and r.area (only returned if polyline is
   *   false) is the area (meters<sup>2</sup>).
   * @description A new vertex is *not* added to the polygon.
   */
  p.PolygonArea.prototype.TestEdge = function(azi, s, reverse, sign) {
    var vals = {number: this.num ? this.num + 1 : 0}, t, tempsum, crossings;
    if (this.num === 0)
      return vals;
    vals.perimeter = this._perimetersum.Sum() + s;
    if (this.polyline)
      return vals;

    tempsum = this._areasum.Sum();
    crossings = this._crossings;
    t = this._geod.Direct(this.lat, this.lon, azi, s, this._mask);
    tempsum += t.S12;
    crossings += transitdirect(this.lon, t.lon2);
    t = this._geod.Inverse(t.lat2, t.lon2, this._lat0, this._lon0, this._mask);
    vals.perimeter += t.s12;
    tempsum += t.S12;
    crossings += transit(t.lon2, this._lon0);

    if (crossings & 1)
      tempsum += (tempsum < 0 ? 1 : -1) * this._area0/2;
    // area is with the clockwise sense.  If !reverse convert to
    // counter-clockwise convention.
    if (!reverse)
      tempsum *= -1;
    // If sign put area in (-area0/2, area0/2], else put area in [0, area0)
    if (sign) {
      if (tempsum > this._area0/2)
        tempsum -= this._area0;
      else if (tempsum <= -this._area0/2)
        tempsum += this._area0;
    } else {
      if (tempsum >= this._area0)
        tempsum -= this._area0;
      else if (tempsum < 0)
        tempsum += this._area0;
    }
    vals.area = tempsum;
    return vals;
  };

})(GeographicLib.PolygonArea, GeographicLib.Geodesic,
   GeographicLib.Math, GeographicLib.Accumulator);
