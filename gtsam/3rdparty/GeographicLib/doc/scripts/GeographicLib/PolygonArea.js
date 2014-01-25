/**
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
 *    http://dx.doi.org/10.1007/s00190-012-0578-z
 *    Addenda: http://geographiclib.sf.net/geod-addenda.html
 *
 * Copyright (c) Charles Karney (2011-2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Load AFTER GeographicLib/Math.js and GeographicLib/Geodesic.js

GeographicLib.PolygonArea = {};

(function() {
  var m = GeographicLib.Math;
  var a = GeographicLib.Accumulator;
  var g = GeographicLib.Geodesic;
  var p = GeographicLib.PolygonArea;

  p.transit = function(lon1, lon2) {
    // Return 1 or -1 if crossing prime meridian in east or west direction.
    // Otherwise return zero.
    // Compute lon12 the same way as Geodesic::Inverse.
    lon1 = m.AngNormalize(lon1);
    lon2 = m.AngNormalize(lon2);
    var lon12 = m.AngDiff(lon1, lon2);
    var cross =
      lon1 < 0 && lon2 >= 0 && lon12 > 0 ? 1 :
      (lon2 < 0 && lon1 >= 0 && lon12 < 0 ? -1 : 0);
    return cross;
  }

  p.PolygonArea = function(earth, polyline) {
    this._earth = earth;
    this._area0 = 4 * Math.PI * earth._c2;
    this._polyline = !polyline ? false : polyline;
    this._mask = g.LATITUDE | g.LONGITUDE | g.DISTANCE |
          (this._polyline ? g.NONE : g.AREA);
    if (!this._polyline)
      this._areasum = new a.Accumulator(0);
    this._perimetersum = new a.Accumulator(0);
    this.Clear();
  }

  p.PolygonArea.prototype.Clear = function() {
    this._num = 0;
    this._crossings = 0;
    if (!this._polyline)
      this._areasum.Set(0);
    this._perimetersum.Set(0);
    this._lat0 = this._lon0 = this._lat1 = this._lon1 = Number.NaN;
  }

  p.PolygonArea.prototype.AddPoint = function(lat, lon) {
    if (this._num == 0) {
      this._lat0 = this._lat1 = lat;
      this._lon0 = this._lon1 = lon;
    } else {
      var t = this._earth.Inverse(this._lat1, this._lon1, lat, lon, this._mask);
      this._perimetersum.Add(t.s12);
      if (!this._polyline) {
        this._areasum.Add(t.S12);
        this._crossings += p.transit(this._lon1, lon);
      }
      this._lat1 = lat;
      this._lon1 = lon;
    }
    ++this._num;
  }

  p.PolygonArea.prototype.AddEdge = function(azi, s) {
    if (this._num) {
      var t = this._earth.Direct(this._lat1, this._lon1, azi, s, this._mask);
      this._perimetersum.Add(s);
      if (!this._polyline) {
        this._areasum.Add(t.S12);
        this._crossings += p.transit(this._lon1, t.lon2);
      }
      this._lat1 = t.lat2;
      this._lon1 = t.lon2;
    }
    ++this._num;
  }

  // return number, perimeter, area
  p.PolygonArea.prototype.Compute = function(reverse, sign) {
    var vals = {number: this._num};
    if (this._num < 2) {
      vals.perimeter = 0;
      if (!this._polyline)
        vals.area = 0;
      return vals;
    }
    if (this._polyline) {
      vals.perimeter = this._perimetersum.Sum();
      return vals;
    }
    var t = this._earth.Inverse(this._lat1, this._lon1, this._lat0, this._lon0,
                                this._mask);
    vals.perimeter = this._perimetersum.Sum(t.s12);
    var tempsum = new a.Accumulator(this._areasum);
    tempsum.Add(t.S12);
    var crossings = this._crossings + p.transit(this._lon1, this._lon0);
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
  }

  // return number, perimeter, area
  p.PolygonArea.prototype.TestPoint = function(lat, lon, reverse, sign) {
    var vals = {number: this._num + 1};
    if (this._num == 0) {
      vals.perimeter = 0;
      if (!this._polyline)
        vals.area = 0;
      return vals;
    }
    vals.perimeter = this._perimetersum.Sum();
    var tempsum = this._polyline ? 0 : this._areasum.Sum();
    var crossings = this._crossings;
    var t;
    for (var i = 0; i < (this._polyline ? 1 : 2); ++i) {
      t = this._earth.Inverse
      (i == 0 ? this._lat1 : lat, i == 0 ? this._lon1 : lon,
       i != 0 ? this._lat0 : lat, i != 0 ? this._lon0 : lon,
       this._mask);
      vals.perimeter += t.s12;
      if (!this._polyline) {
        tempsum += t.S12;
        crossings += p.transit(i == 0 ? this._lon1 : lon,
                               i != 0 ? this._lon0 : lon);
      }
    }

    if (this._polyline)
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
  }

  // return number, perimeter, area
  p.PolygonArea.prototype.TestEdge = function(azi, s, reverse, sign) {
    var vals = {number: this._num ? this._num + 1 : 0};
    if (this._num == 0)
      return vals;
    vals.perimeter = this._perimetersum.Sum() + s;
    if (this._polyline)
      return vals;

    var tempsum = this._areasum.Sum();
    var crossings = this._crossings;
    var t;
    t = this._earth.Direct(this._lat1, this._lon1, azi, s, this._mask);
    tempsum += t.S12;
    crossings += p.transit(this._lon1, t.lon2);
    t = this._earth(t.lat2, t.lon2, this._lat0, this._lon0, this._mask);
    perimeter += t.s12;
    tempsum += t.S12;
    crossings += p.transit(t.lon2, this._lon0);

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
  }

  p.PolygonArea.prototype.CurrentPoint = function() {
    var vals = {lat: this._lat1, lon: this._lon1};
    return vals;
  }

  p.Area = function(earth, points, polyline) {
    var poly = new p.PolygonArea(earth, polyline);
    for (var i = 0; i < points.length; ++i)
      poly.AddPoint(points[i].lat, points[i].lon);
    return poly.Compute(false, true);
  }

})();
